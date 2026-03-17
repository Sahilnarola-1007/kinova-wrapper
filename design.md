# KinovaInterface Wrapper — Design Document

## Why This Wrapper Exists

The Kortex API requires:
- 40-60 lines of boilerplate to establish a connection
- Manual `delete` for every object (leak-prone)
- Verbose protobuf `mutable_` chains for simple operations
- No real error handling or recovery
- Hardcoded IP/credentials scattered in `main()`

This wrapper reduces all of that to clean, safe, single-line calls.

---

## Architecture

```
User Code
  │
  ▼
KinovaInterface  (your wrapper — one class, owns everything)
  │
  ├── TransportClientTcp  (unique_ptr)
  ├── RouterClient        (unique_ptr)
  ├── SessionManager      (unique_ptr)
  └── BaseClient          (unique_ptr)
        │
        ▼
    Kinova Gen3 Hardware (over TCP/IP)
```

## Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Singleton or multi-instance? | Single instance, NOT singleton | One robot, but singleton makes testing harder |
| Error strategy | `bool` returns for expected failures, exceptions for unrecoverable errors | Consistent, caller decides how to handle |
| Thread model | Single `std::mutex` protecting all Kortex calls | Kortex API is not thread-safe |
| Sync vs async | Both — sync by default, async variants with `std::future` | Sync is simpler, async needed for non-blocking motion |
| Units | Radians in public API, degrees internally (Kortex convention) | Radians = robotics standard, convert inside wrapper |
| Joint limits | Queried from robot at connect time, not hardcoded | Limits vary by firmware version |

---

## Class Declaration (Overview)

```cpp
class KinovaInterface {
public:
    KinovaInterface();
    ~KinovaInterface();

    // Non-copyable, non-movable (owns hardware connection)
    KinovaInterface(const KinovaInterface&) = delete;
    KinovaInterface& operator=(const KinovaInterface&) = delete;

    // --- Connection ---
    bool connect(const std::string& ip_address, uint32_t port = 10000);
    void disconnect();
    bool isConnected() const;

    // --- Motion ---
    bool moveToJointAngles(const std::vector<double>& angles);                   // sync
    std::future<bool> moveToJointAnglesAsync(const std::vector<double>& angles);  // async
    bool moveToCartesianPose(const Pose& pose);                                   // sync
    std::future<bool> moveToCartesianPoseAsync(const Pose& pose);                 // async

    // --- State ---
    std::vector<double> getJointAngles();
    Pose getCurrentPose();
    std::vector<double> getWrench();  // forces/torques from F/T sensor (6 values)

    // --- Safety ---
    void emergencyStop();
    bool clearEmergencyStop();
    bool setSpeedLimit(double fraction);  // 0.0 to 1.0

private:
    std::unique_ptr<k_api::TransportClientTcp> transport_;
    std::unique_ptr<k_api::RouterClient> router_;
    std::unique_ptr<k_api::SessionManager> session_manager_;
    std::unique_ptr<k_api::Base::BaseClient> base_client_;

    mutable std::mutex mutex_;         // protects all Kortex calls
    std::atomic<bool> connected_{false};
    std::atomic<bool> e_stop_active_{false};

    // Joint limits — queried from robot during connect()
    // Joints 1,3,5,7 are continuous rotation (no hard limits)
    // Joints 2,4,6 have software limits to prevent self-collision
    std::vector<double> joint_min_limits_;  // populated during connect()
    std::vector<double> joint_max_limits_;  // populated during connect()

    bool validateJointAngles(const std::vector<double>& angles) const;
};
```

---

## Function Specifications

### 1. connect()

```
Signature:  bool connect(const std::string& ip_address, uint32_t port = 10000)

Purpose:    Establish connection to Kinova Gen3

Parameters:
  ip_address — robot's IP (e.g., "192.168.1.10")
  port       — TCP port, defaults to 10000

Returns:
  true  — connected, session active, ready for commands
  false — failed

Internal steps:
  1. Lock mutex_
  2. If already connected → disconnect first
  3. Create TransportClientTcp → store in transport_ (unique_ptr)
  4. Call transport_->connect(ip, port)
  5. Create RouterClient(transport_.get()) → store in router_
  6. Create SessionManager(router_.get()) → store in session_manager_
  7. Build CreateSessionInfo with credentials
  8. Call session_manager_->CreateSession(session_info)
  9. Create BaseClient(router_.get()) → store in base_client_
  10. Query joint limits from robot → store in joint_min_limits_, joint_max_limits_
  11. Set connected_ = true
  If ANY step fails → reset all unique_ptrs (auto cleanup), return false

Errors:
  - Robot unreachable → return false, log "Cannot reach {ip}:{port}"
  - Auth failed → return false, log "Authentication failed"
  - Already connected → auto-disconnect, then reconnect

Thread safety: Locks mutex_ for entire operation
```

### 2. disconnect()

```
Signature:  void disconnect()

Purpose:    Cleanly close connection and release all resources

Parameters: none
Returns:    void (cleanup must not throw — destructor calls this)

Internal steps:
  1. Lock mutex_
  2. If not connected → return immediately
  3. Close session via session_manager_
  4. Reset base_client_ (unique_ptr reset → auto delete)
  5. Reset session_manager_
  6. Reset router_
  7. Disconnect transport, then reset transport_
  8. Set connected_ = false
  Order matters: reverse of construction order

Errors:
  - None thrown. If session close fails, log warning but continue cleanup.
  - Destructor calls disconnect(), so this must be noexcept-safe.

Thread safety: Locks mutex_
```

### 3. isConnected()

```
Signature:  bool isConnected() const

Purpose:    Check if robot connection is active

Parameters: none
Returns:    true if connected, false otherwise

Internal steps:
  1. Return connected_.load()  (atomic read, no mutex needed)

Errors: none

Thread safety: Atomic variable, lock-free
```

### 4. moveToJointAngles()

```
Signature:  bool moveToJointAngles(const std::vector<double>& angles)

Purpose:    Move robot to specified joint angles. Blocks until motion completes.

Parameters:
  angles — exactly 7 values in radians, one per joint

Returns:
  true  — motion completed successfully
  false — failed (invalid angles, not connected, e-stop active, motion error)

Internal steps:
  1. Check connected_ → return false if not connected
  2. Check e_stop_active_ → return false if e-stop is on
  3. Call validateJointAngles(angles) → return false if angles out of limits
  4. Lock mutex_
  5. Convert radians to degrees (Kortex expects degrees)
  6. Build protobuf Action message:
     - Create Action
     - mutable_reach_joint_angles() → mutable_joint_angles()
     - Loop: Add() each joint with identifier and value
  7. Call base_client_->ExecuteAction(action)
  8. Wait for action completion (poll or notification)
  9. Return true on success

Errors:
  - Wrong size vector (not 7) → return false, log "Expected 7 angles, got {n}"
  - Angle out of limits → return false, log "Joint {i}: {value} exceeds limit [{min}, {max}]"
  - Not connected → return false, log "Not connected"
  - E-stop active → return false, log "Emergency stop is active"
  - Motion failed → return false, log Kortex error message

Thread safety: Locks mutex_ for Kortex calls
```

### 5. moveToJointAnglesAsync()

```
Signature:  std::future<bool> moveToJointAnglesAsync(const std::vector<double>& angles)

Purpose:    Same as moveToJointAngles, but non-blocking. Returns immediately.

Parameters: same as moveToJointAngles
Returns:    std::future<bool> — caller can poll with wait_for() or block with get()

Internal steps:
  1. Validate angles (same checks as sync version)
  2. Launch std::async(std::launch::async, [this, angles]{ return moveToJointAngles(angles); })
  3. Return the future

Usage by caller:
  auto future = kinova.moveToJointAnglesAsync({0.26, 0.52, 0, -1.05, 0, -0.78, 0});
  // ... do other work ...
  bool success = future.get();  // blocks until motion done

Thread safety: The async thread will lock mutex_ internally via moveToJointAngles
```

### 6. moveToCartesianPose()

```
Signature:  bool moveToCartesianPose(const Pose& pose)

Purpose:    Move end-effector to a Cartesian position + orientation

Parameters:
  pose — struct containing:
    double x, y, z          (meters, position)
    double theta_x, theta_y, theta_z  (degrees, orientation — Kortex convention)

Returns:
  true/false — same pattern as moveToJointAngles

Internal steps:
  1. Connection and e-stop checks
  2. Lock mutex_
  3. Build protobuf Action with mutable_reach_pose()
  4. Set target position and orientation
  5. Execute and wait
  6. Return result

Note: Kortex handles inverse kinematics internally.

Errors:
  - Pose unreachable (outside workspace) → return false
  - Same connection/e-stop errors as moveToJointAngles
```

### 7. getJointAngles()

```
Signature:  std::vector<double> getJointAngles()

Purpose:    Read current joint positions from robot

Parameters: none
Returns:    vector of 7 doubles (radians) — empty vector on failure

Internal steps:
  1. Check connected_
  2. Lock mutex_
  3. Call base_client_->GetMeasuredJointAngles()
  4. Extract values, convert degrees (Kortex) to radians
  5. Return by value (compiler optimizes via move — no copy)

Errors:
  - Not connected → return empty vector, log error
  - Communication error → return empty vector, log error

Why return by value?
  Returning std::vector by value triggers Return Value Optimization (RVO).
  The compiler constructs the vector directly in the caller's memory.
  No copy happens. Clean API, zero overhead.
```

### 8. getCurrentPose()

```
Signature:  Pose getCurrentPose()

Purpose:    Read current end-effector position and orientation

Parameters: none
Returns:    Pose struct (x, y, z, theta_x, theta_y, theta_z) — zeroed on failure

Internal steps:
  1. Check connected_
  2. Lock mutex_
  3. Call base_client_->GetMeasuredCartesianPose()
  4. Fill Pose struct from response
  5. Return by value

Errors: same as getJointAngles
```

### 9. getWrench()

```
Signature:  std::vector<double> getWrench()

Purpose:    Read forces (Fx, Fy, Fz) and torques (Tx, Ty, Tz) from built-in sensor

Parameters: none
Returns:    vector of 6 doubles — empty vector on failure

Errors: same as getJointAngles
```

### 10. emergencyStop()

```
Signature:  void emergencyStop()

Purpose:    Immediately stop all robot motion

Parameters: none
Returns:    void — must ALWAYS attempt to stop, never throw

Internal steps:
  1. Set e_stop_active_ = true (atomic, instant)
  2. Lock mutex_
  3. Call base_client_->ApplyEmergencyStop()
  4. Log "Emergency stop activated"

Why void and not bool?
  E-stop is a safety function. The caller should never have to check
  if it "worked" — it must always be attempted. If the Kortex call
  fails (e.g., disconnected), the e_stop_active_ flag still prevents
  any further commands through the wrapper.

Errors:
  - If Kortex call fails → log error, but e_stop_active_ flag is still set
  - All subsequent motion commands will be rejected until clearEmergencyStop()

Thread safety: Atomic flag + mutex
```

### 11. clearEmergencyStop()

```
Signature:  bool clearEmergencyStop()

Purpose:    Reset e-stop flag, allow commands again

Parameters: none
Returns:    true if cleared successfully, false if ClearFaults failed

Internal steps:
  1. Lock mutex_
  2. Call base_client_->ClearFaults()
  3. Set e_stop_active_ = false
  4. Log "Emergency stop cleared"

Errors:
  - ClearFaults fails → return false, log error, keep e_stop_active_ = true
```

### 12. setSpeedLimit()

```
Signature:  bool setSpeedLimit(double fraction)

Purpose:    Limit robot speed as a fraction of maximum

Parameters:
  fraction — 0.0 (stopped) to 1.0 (full speed)

Returns:
  true if applied, false if invalid value or not connected

Internal steps:
  1. Validate: 0.0 <= fraction <= 1.0
  2. Lock mutex_
  3. Apply via Kortex speed limit API
  4. Store current limit for reference

Errors:
  - fraction out of range → return false
  - Not connected → return false
```

### 13. validateJointAngles() (private)

```
Signature:  bool validateJointAngles(const std::vector<double>& angles) const

Purpose:    Check that angles are valid before sending to robot

Parameters:
  angles — vector to validate (in radians)

Returns:
  true if valid, false if not

Checks:
  1. angles.size() == 7
  2. Convert each angle to degrees, check within [joint_min_limits_[i], joint_max_limits_[i]]
     (limits stored in degrees, matching Kortex convention)

This is private — called internally by moveToJointAngles()
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

## Usage Example

```cpp
// WITHOUT wrapper (Kortex raw) — ~60 lines
auto transport = new TransportClientTcp();
transport->connect(ip, port);
auto router = new RouterClient(transport, ...);
auto session_mgr = new SessionManager(router);
auto session_info = CreateSessionInfo();
session_info.set_username("admin");
session_info.set_password("admin");
session_mgr->CreateSession(session_info);
auto base = new Base::BaseClient(router);
// ... build protobuf, send command, manual cleanup ...
delete base; delete session_mgr; delete router; delete transport;

// WITH wrapper — 3 lines
KinovaInterface kinova;
kinova.connect("192.168.1.10");
kinova.moveToJointAngles({0, 0.26, 0, -1.05, 0, -0.78, 0});  // radians
// destructor auto-cleans everything
```

---

## Destruction Order

```
~KinovaInterface() calls disconnect(), which resets in reverse order:
  base_client_.reset()       // 4th created → 1st destroyed
  session_manager_.reset()   // 3rd created → 2nd destroyed
  router_.reset()            // 2nd created → 3rd destroyed
  transport_.reset()         // 1st created → 4th destroyed
```

This is RAII — resources released automatically, in the correct order, even if an exception occurs.
