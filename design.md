# KinovaInterface Wrapper — Design Document

## Why This Wrapper Exists

The Kortex SDK provides low-level access to the Kinova Gen3 but imposes significant cost on every consumer:

| Pain Point | Raw Kortex SDK | This Wrapper |
|------------|---------------|--------------|
| Connection setup | 40-60 lines of boilerplate per program | `kinova.connect("192.168.1.10")` |
| Resource cleanup | Manual `delete` on 4 objects, leak-prone on exceptions | RAII — destructor auto-cleans, exception-safe |
| Motion command | 15+ lines of protobuf `mutable_` chain construction | `kinova.moveToJointAngles({...})` |
| Gripper command | Build protobuf, send non-blocking, poll loop, extract from repeated field | `kinova.closeGripper(40.0)` |
| Thread safety | None — caller's responsibility | Built-in `std::mutex` on every call |
| Input validation | None — SDK crashes on empty IP, ignores bad joint angles | Validates before touching SDK |
| Error handling | Exceptions only, no graceful fallback | `bool` returns for expected failures, exceptions for unrecoverable |
| Testing without hardware | Not possible | Mock SDK via `#ifdef`, 25 Google Tests run without robot |

The wrapper sits between application code (ROS2 nodes, pick-and-place logic, RL policies) and the raw SDK. Application code never touches Kortex directly.

---

## Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│  Application Layer                                               │
│                                                                  │
│  ROS2 Lifecycle Node ─── Action Server ─── Perception Pipeline   │
│  (KinovaLifecycleController)  (MoveToPose)    (future: Week 5+)  │
└────────────────────────────────┬─────────────────────────────────┘
                                 │
                                 ▼
┌──────────────────────────────────────────────────────────────────┐
│  KinovaInterface (this wrapper)                                  │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  Public API (what callers see)                             │  │
│  │                                                            │  │
│  │  connect/disconnect    moveToJointAngles    openGripper    │  │
│  │  isConnected           moveToCartesianPose  closeGripper   │  │
│  │                        async variants       setGripperPos  │  │
│  │  getJointAngles        emergencyStop        getGripperPos  │  │
│  │  getCurrentPose        clearEmergencyStop   isObjectDetect │  │
│  │  getWrench             setSpeedLimit                       │  │
│  │  setCartesianVelocity  stopMotion           isVelocActive  │  │
│  └────────────────────────────────────────────────────────────┘  │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  Internal Infrastructure (hidden from callers)             │  │
│  │                                                            │  │
│  │  std::mutex ──── protects all Kortex calls                 │  │
│  │  std::atomic ─── connected_, e_stop_active_ (lock-free)    │  │
│  │  unique_ptr ──── owns transport, router, session, base     │  │
│  │  validation ──── joint limits, gripper range, connection    │  │
│  │  conversion ──── radians↔degrees at API boundary           │  │
│  │  watchdog ────── auto-stop velocity on control loop death   │  │
│  └────────────────────────────────────────────────────────────┘  │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  SDK Object Ownership (RAII, reverse destruction)          │  │
│  │                                                            │  │
│  │  unique_ptr<TransportClientTcp> ──→ TCP socket             │  │
│  │  unique_ptr<RouterClient> ────────→ message routing        │  │
│  │  unique_ptr<SessionManager> ──────→ auth session           │  │
│  │  unique_ptr<BaseClient> ──────────→ motion/gripper/state   │  │
│  │                                                            │  │
│  │  Creation:    transport → router → session → base          │  │
│  │  Destruction: base → session → router → transport          │  │
│  └────────────────────────────────────────────────────────────┘  │
└────────────────────────────────┬─────────────────────────────────┘
                                 │  protobuf over TCP/IP
                                 ▼
┌──────────────────────────────────────────────────────────────────┐
│  Kortex SDK (Kinova's closed-source library)                     │
│  Protobuf-based RPC to robot hardware                            │
└────────────────────────────────┬─────────────────────────────────┘
                                 │
                                 ▼
┌──────────────────────────────────────────────────────────────────┐
│  Kinova Gen3 7-DOF + Robotiq 2F-85                               │
│  192.168.1.10:10000 (default)                                    │
└──────────────────────────────────────────────────────────────────┘
```

---

## Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Singleton or multi-instance? | Single instance, NOT singleton | One robot, but singleton makes testing harder |
| Error strategy | `bool` returns for expected failures, exceptions for unrecoverable | Consistent, caller decides how to handle |
| Thread model | Single `std::mutex` protecting all Kortex calls | Kortex API is not thread-safe; one lock is simple and correct |
| Sync vs async | Both — sync by default, async variants with `std::future` | Sync is simpler, async needed for non-blocking motion |
| Units | Radians in public API, degrees internally (Kortex convention) | Radians = robotics standard, convert at API boundary |
| Joint limits | Hardcoded defaults matching Gen3 specs | `GetJointLimits()` unavailable in our SDK build |
| Gripper control | Position mode + polling loop with timeout | `SendGripperCommand` is non-blocking; must poll completion |
| Object detection | Commanded vs actual position gap | Robotiq has no boolean sensor; stall = motor current spike = position gap |
| Testability | `#ifdef USE_KORTEX_MOCK` compile-time switch | Mock mirrors real SDK types; tests run without hardware |
| Copy/Move semantics | Deleted (non-copyable, non-movable) | Object owns hardware connection; copy = two owners = dangerous |
| Velocity safety | Clamp + workspace boundary + watchdog | Three-layer defense: clamp prevents excessive speed, boundary prevents leaving safe volume, watchdog prevents runaway on control loop failure |

---

## Class Declaration

```cpp
class KinovaInterface {
public:
    KinovaInterface();
    ~KinovaInterface();

    KinovaInterface(const KinovaInterface&) = delete;
    KinovaInterface& operator=(const KinovaInterface&) = delete;
    KinovaInterface(KinovaInterface&&) = delete;
    KinovaInterface& operator=(KinovaInterface&&) = delete;

    // --- Connection ---
    bool connect(const std::string& ip_address, uint32_t port = 10000,
                 const std::string& username = "admin",
                 const std::string& password = "admin");
    void disconnect();
    bool isConnected() const;

    // --- Motion ---
    bool moveToJointAngles(const std::vector<double>& angles);
    std::future<bool> moveToJointAnglesAsync(const std::vector<double>& angles);
    bool moveToCartesianPose(const Pose& pose);
    std::future<bool> moveToCartesianPoseAsync(const Pose& pose);

    // --- Gripper (Robotiq 2F-85) ---
    bool openGripper(double speed = 0.1);
    bool closeGripper(double force = 40.0, double speed = 0.1);
    bool setGripperPosition(double position, double speed = 0.1);
    double getGripperPosition();
    bool isObjectDetected();

    // --- State ---
    std::vector<double> getJointAngles();
    Pose getCurrentPose();
    std::vector<double> getWrench();


    // --- Safety ---
    void emergencyStop();
    bool isEStopActive() const;
    bool clearEmergencyStop();
    bool setSpeedLimit(double fraction);

    // --- Velocity ---
    bool setCartesianVelocity(double vx, double vy, double vz,
                              double wx, double wy, double wz);
    void stopMotion();
    bool isVelocityActive() const;

private:
    std::unique_ptr<k_api::TransportClientTcp> transport_;
    std::unique_ptr<k_api::RouterClient> router_;
    std::unique_ptr<k_api::SessionManager> session_manager_;
    std::unique_ptr<k_api::Base::BaseClient> base_client_;

    mutable std::mutex mutex_;
    std::atomic<bool> connected_{false};
    std::atomic<bool> e_stop_active_{false};

    std::vector<double> joint_min_limits_;
    std::vector<double> joint_max_limits_;

    double current_speed_fraction_ = 1.0;
    double last_commanded_grip_pos_{0.0};

    bool validateJointAngles(const std::vector<double>& angles) const;
    void disconnectLocked();  // assumes mutex_ already held

    static constexpr int kNumJoints = 7;
    static constexpr double kRadToDeg = 180.0 / M_PI;
    static constexpr double kDegToRad = M_PI / 180.0;
    static constexpr double kGripperTimeoutSec = 5.0;
    static constexpr double kGripperPositionTolerance = 0.01;
    static constexpr double kMaxGripperForceN = 235.0;

    // --- Velocity ---
    static constexpr double kMaxLinearVelocity = 0.5;   // m/s
    static constexpr double kMaxAngularVelocity = 40.0;  // deg/s
    static constexpr double kWatchdogTimeoutMs = 100.0;

    // Workspace boundary (meters from base)
    static constexpr double kWorkspaceXMin = 0.15, kWorkspaceXMax = 0.8;
    static constexpr double kWorkspaceYMin = -0.13, kWorkspaceYMax = 0.5;
    static constexpr double kWorkspaceZMin = 0.03, kWorkspaceZMax = 1.0;

    std::thread watchdog_thread_;
    std::atomic<bool> watchdog_running_{false};
    std::atomic<bool> velocity_active_{false};
    std::chrono::steady_clock::time_point last_velocity_time_;
    std::mutex velocity_time_mutex_;  // protects last_velocity_time_ only
};
```

---

## Function Specifications

### connect()

```
bool connect(ip_address, port=10000, username="admin", password="admin")

Steps:
  1. Lock mutex_
  2. Validate IP not empty
  3. If already connected → disconnectLocked() first
  4. Create TransportClientTcp → connect(ip, port)
  5. Create RouterClient(transport_)
  6. Create SessionManager(router_) → CreateSession(credentials)
  7. Create BaseClient(router_)
  8. Set connected_ = true, e_stop_active_ = false
  If ANY step fails → reset all unique_ptrs (auto cleanup), return false
```

### disconnect() / disconnectLocked()

```
disconnect():       Locks mutex_, calls disconnectLocked()
disconnectLocked(): Assumes mutex_ held. Resets in reverse order:
                    base → session → router → transport
                    Sets connected_ = false. Never throws.

Why two functions?
  connect() already holds mutex_ when it needs to disconnect.
  std::mutex is not reentrant — calling disconnect() from inside
  connect() would deadlock. disconnectLocked() skips the lock.
```

### moveToJointAngles()

```
bool moveToJointAngles(angles)

Checks:     connected_, e_stop_active_, validateJointAngles(angles)
Action:     Converts radians→degrees, builds protobuf Action,
            sets SINGLE_LEVEL_SERVOING (real SDK), calls ExecuteAction()
Returns:    true on success, false on any failure
```

### moveToCartesianPose()

```
bool moveToCartesianPose(pose)

Same pattern as moveToJointAngles but uses mutable_reach_pose().
Kortex handles inverse kinematics internally.
```

### Async Variants

```
moveToJointAnglesAsync() / moveToCartesianPoseAsync()

Launch std::async(std::launch::async, ...) capturing args by value.
Return std::future<bool>. Thread acquires mutex_ via the sync method.
```

### setGripperPosition()

```
bool setGripperPosition(position, speed=0.1)

Purpose:    Move Robotiq 2F-85 to target. Blocks until reached or timeout.

Steps:
  1. Store last_commanded_grip_pos_ (for object detection)
  2. Validate: connected_, e_stop_active_, position in [0.0, 1.0]
  3. Lock mutex_
  4. Build GripperCommand: mode=POSITION, finger_id=1, value=position
  5. Build GripperRequest: mode=POSITION (for readback)
  6. Read initial position via GetMeasuredGripperMovement()
  7. Send command via SendGripperCommand() — NON-BLOCKING
  8. Poll loop (100ms):
     - Read position, check finger_size() > 0
     - |current - target| < tolerance → return true
     - elapsed > 5s → return false (timeout)

Key: protobuf objects are LOCAL (not class members) to prevent
add_finger() accumulation across calls.
```

### openGripper() / closeGripper()

```
openGripper(speed):          return setGripperPosition(0.0, speed)
closeGripper(force, speed):  return setGripperPosition(1.0, speed)

force parameter: TODO — not yet wired to Kortex.
```

### getGripperPosition()

```
double getGripperPosition()

Returns [0.0, 1.0] on success, -1.0 on failure.
No logging — called frequently by polling loops and isObjectDetected().
```

### isObjectDetected()

```
bool isObjectDetected()

Logic:
  1. current = getGripperPosition()
  2. If current < 0 → return false (read error)
  3. If (last_commanded - current) > tolerance AND last_commanded > 0.5
     → return true (stalled on object)

Why last_commanded > 0.5?
  Only relevant when closing. Opening doesn't stall on objects.

Why (commanded - actual)?
  Commanded 1.0, actual 0.4 → gap 0.6 → object detected.
```

### TrajectoryPoint struct
joint_angles: vector of 7 doubles (radians)
time_from_start: seconds from trajectory start

### executeTrajectory()
Signature: bool executeTrajectory(const vector<TrajectoryPoint>& waypoints, MotionCallback feedback_cb = nullptr)
- Validates all waypoints (joint limits, monotonic timing)
- Executes sequentially via Kortex ExecuteAction per waypoint
- Fires optional feedback callback after each waypoint with current joints and progress
- Thread-safe: holds mutex_ for entire execution

### executeTrajectoryAsync()
- std::async wrapper around executeTrajectory
- Captures waypoints and callback by value

### validateTrajectory() (private)
- Empty check, first time > 0, monotonic times, joint limit validation
- Called under mutex_, never locks itself

### setCartesianVelocity()

```
bool setCartesianVelocity(vx, vy, vz, wx, wy, wz)

Purpose:    Send continuous Cartesian velocity command. Non-blocking.
            Foundation for force control (Week 4) and visual servoing (Week 16).

Steps:
  1. Check connected_, e_stop_active_
  2. Clamp linear components to ±0.5 m/s, angular to ±40 deg/s
  3. Log warning if any clipping occurred (upstream gain tuning signal)
  4. Lock mutex_
  5. Read current EE pose via GetMeasuredCartesianPose()
  6. Workspace boundary check — per-axis:
     If EE at/beyond boundary AND velocity pushes further out → zero that component
     Allows motion AWAY from boundary, blocks motion INTO it
  7. Build TwistCommand (mock: direct assign, real: protobuf setters)
  8. SendTwistCommand()
  9. Unlock mutex_
  10. Update last_velocity_time_ (under velocity_time_mutex_)
  11. Start watchdog thread on first call

Returns: true on success, false if disconnected/e-stop/SDK error
```

### stopMotion()

```
void stopMotion()

Purpose:    Zero all velocities. Arm stops but remains powered and ready.
            Unlike emergencyStop(), no clearing required before next command.

Steps:
  1. Set velocity_active_ = false (tell watchdog to stop monitoring)
  2. Lock mutex_
  3. Call base_client_->Stop()

No precondition checks — safety function, must always attempt to stop.
```

### isVelocityActive()

```
bool isVelocityActive()

Lock-free read of velocity_active_ atomic. Returns true if arm is in
velocity control mode (between first setCartesianVelocity() and
stopMotion()/watchdog timeout).
```

### Watchdog Design

```
Problem:
  Velocity commands are continuous — the arm keeps moving at the last
  commanded velocity. If the control loop crashes, hangs, or loses
  connection, no one sends stopMotion(). The arm drifts indefinitely.

Solution:
  Background thread checks every 50ms:
    if (velocity_active_ && now - last_velocity_time_ > 100ms)
      → call stopMotion(), set velocity_active_ = false

Lifecycle:
  - Created on first setCartesianVelocity() call
  - Runs until watchdog_running_ set to false
  - Destructor sets watchdog_running_ = false, then joins thread

Why separate velocity_time_mutex_?
  mutex_ protects Kortex SDK calls and can be held for seconds
  (e.g., during moveToJointAngles). If the watchdog locked mutex_
  to read the timestamp, it would block behind slow motion commands
  and fail to detect a dead velocity loop. velocity_time_mutex_
  protects only last_velocity_time_ — locked for microseconds.

Worst case drift:
  100ms timeout + 50ms sleep period = ~150ms before auto-stop.
  At max velocity (0.5 m/s), that's ~7.5cm of drift.
  Workspace boundaries provide secondary protection.
```

### Workspace Boundary

```
A rectangular safety volume around the arm's operating space.
Prevents velocity commands from driving the EE outside safe limits.

Current limits (tuned to lab setup):
  X: [0.15, 0.80] m    Y: [-0.13, 0.50] m    Z: [0.03, 1.00] m

Logic: per-axis check BEFORE sending velocity command.
  if (EE.x >= X_max && vx > 0) → vx = 0   (block outward motion)
  if (EE.x <= X_min && vx < 0) → vx = 0   (block outward motion)
  // ... same for Y and Z

Key: motion AWAY from boundary is always allowed. This prevents
the arm from getting stuck at a boundary with no way to recover.

Z_min = 0.03m (3cm above table) is the most critical — prevents
table crashes during downward velocity commands.
```

### emergencyStop()

```
void emergencyStop()

Sets e_stop_active_ = true FIRST (atomic, instant).
Then locks mutex_, calls ApplyEmergencyStop().
Never throws — safety function must always attempt to stop.
```

### State Reading

```
getJointAngles():   Returns 7 doubles in radians. Empty on failure.
getCurrentPose():   Returns Pose struct. Zeroed on failure.
getWrench():        Returns empty (TODO — not available in SDK build).

All follow: check connected_ → lock mutex_ → call SDK → convert → return by value (RVO).
```

---

## Pose Struct

```cpp
struct Pose {
    double x = 0.0;        // meters
    double y = 0.0;        // meters
    double z = 0.0;        // meters
    double theta_x = 0.0;  // degrees (Kortex convention)
    double theta_y = 0.0;  // degrees
    double theta_z = 0.0;  // degrees
};
```

---

## Kortex SDK Quirks (Discovered During Development)

| Issue | Root Cause | Solution |
|-------|-----------|----------|
| Protobuf command objects accumulate | `add_finger()` appends to repeated field each call | Use local variables, never class members |
| `SendGripperCommand` is non-blocking | SDK design — returns immediately | Poll with `GetMeasuredGripperMovement` in loop |
| `finger(0)` vs `set_finger_identifier(1)` | Protobuf array index (0-based) vs Kinova actuator ID (1-based) | Always `add_finger()` once, ID=1, read `finger(0)` |
| `bad_function_call` on real robot connect | Closed-source SDK internal thread | No fix possible, no functional impact |
| `SetServoingMode` required before `ExecuteAction` | Real SDK needs explicit mode switch | Guard with `#ifndef USE_KORTEX_MOCK` |
| Empty IP crashes SDK | SDK doesn't validate inputs internally | Validate in wrapper before touching SDK |
| `GetJointLimits` unavailable | Not in our SDK build | Hardcode limits from Gen3 datasheet |
| `GetMeasuredWrench` unavailable | Not in our SDK build | Return empty vector, implement when SDK updated |
| `CreateSessionInfo` differs mock vs real | Real uses `Session::CreateSessionInfo` + `set_username()` | `#ifdef` guard for struct type and accessors |
| `RouterClient` constructor differs | Real takes error callback lambda, mock doesn't | `#ifdef` guard for constructor call |
| `SendTwistCommand` is non-blocking | SDK design — arm moves continuously at last velocity | Must implement watchdog auto-stop; `Base::Stop()` zeroes velocity |
| `TwistCommand` struct differs mock vs real | Real uses protobuf `set_reference_frame()` + `mutable_twist()` | `#ifdef` guard, same pattern as all other commands |
KORTEX SESSION TIMEOUT (critical):
Always set session_inactivity_timeout=60000 and 
connection_inactivity_timeout=2000 in CreateSession.
Without this, sessions die during wait_for() and 
robot aborts motion silently.

---

## Known Issues

### `bad_function_call` on activation (Kortex SDK)
A single `bad_function_call` message prints to stderr immediately after
connecting to the real Gen3. This originates from a Kortex SDK internal
thread — not from wrapper or ROS2 code. Evidence:

- Try-catch blocks in both `on_activate()` and `publishJointStates()` do not catch it
- Appears exactly once (not from 100ms timer loop)
- No ROS2 log prefix — raw stderr from SDK internals
- No functional impact: joint data publishes, all transitions succeed

Kortex SDK is closed-source (.so), so the root cause cannot be patched.
No action required unless it causes actual failures.
