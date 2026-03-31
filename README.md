# KinovaInterface

A production-grade C++ wrapper around the Kinova Kortex API for the Gen3 7-DOF robotic arm with Robotiq 2F-85 gripper. Built with modern C++17 patterns (RAII, smart pointers, thread safety) and designed as the control foundation for a visual servoing pick-and-place capstone.

## The Problem

The Kortex SDK is powerful but requires significant boilerplate for every operation. A simple "connect and move" sequence looks like this:

```cpp
// Raw Kortex: ~60 lines for a single motion command
auto* transport = new TransportClientTcp();
transport->connect("192.168.1.10", 10000);
auto* router = new RouterClient(transport, [](auto err){ /* error cb */ });
auto* session_mgr = new SessionManager(router);
auto session_info = Session::CreateSessionInfo();
session_info.set_username("admin");
session_info.set_password("admin");
session_mgr->CreateSession(session_info);
auto* base = new Base::BaseClient(router);

// Build protobuf action (another 15+ lines)
Base::Action action;
auto* angles = action.mutable_reach_joint_angles()->mutable_joint_angles();
for (int i = 0; i < 7; i++) {
    auto* j = angles->add_joint_angles();
    j->set_joint_identifier(i);
    j->set_value(target[i]);
}
auto mode = Base::ServoingModeInformation();
mode.set_servoing_mode(Base::ServoingMode::SINGLE_LEVEL_SERVOING);
base->SetServoingMode(mode);
base->ExecuteAction(action);

// Manual cleanup — leaks if any step above throws
delete base;
delete session_mgr;
delete router;
transport->disconnect();
delete transport;
```

Every project repeats this. Errors are silently ignored. Resources leak on exceptions. Credentials are hardcoded. There is no thread safety, no input validation, and no way to test without hardware.

## The Solution

```cpp
KinovaInterface kinova;
kinova.connect("192.168.1.10");
kinova.moveToJointAngles({0, 0.26, 0, -1.05, 0, -0.78, 0});
kinova.closeGripper(40.0);

if (kinova.isObjectDetected()) {
    // object grasped — proceed with lift
}
// destructor auto-cleans everything, even on exceptions
```

Three lines replace sixty. Resources are managed automatically via RAII. Every call is thread-safe, input-validated, and covered by 25 automated tests.

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                      User Code / ROS2                        │
│  (Lifecycle nodes, action servers, pick-and-place logic)     │
└──────────────────────────┬───────────────────────────────────┘
                           │  clean, single-line calls
                           ▼
┌──────────────────────────────────────────────────────────────┐
│                    KinovaInterface                           │
│                                                              │
│  ┌─────────────┐ ┌──────────────┐ ┌────────────────────┐    │
│  │ Connection   │ │ Motion       │ │ Gripper            │    │
│  │ Manager      │ │ Controller   │ │ Controller         │    │
│  │              │ │              │ │                    │    │
│  │ connect()    │ │ moveToJoint  │ │ openGripper()      │    │
│  │ disconnect() │ │  Angles()   │ │ closeGripper()     │    │
│  │ isConnected()│ │ moveToPose() │ │ setGripperPos()    │    │
│  │              │ │ async vars   │ │ isObjectDetected() │    │
│  └─────────────┘ └──────────────┘ └────────────────────┘    │
│                                                              │
│  ┌─────────────┐ ┌──────────────┐ ┌────────────────────┐    │
│  │ State        │ │ Safety       │ │ Validation         │    │
│  │ Reader       │ │ System       │ │                    │    │
│  │              │ │              │ │ Joint limits       │    │
│  │ getJoint     │ │ eStop()      │ │ Gripper range      │    │
│  │  Angles()   │ │ clearEStop() │ │ Connection check   │    │
│  │ getPose()    │ │ speedLimit() │ │ E-stop gate        │    │
│  │ getWrench()  │ │ atomic flags │ │                    │    │
│  └─────────────┘ └──────────────┘ └────────────────────┘    │
│                                                              │
│  Thread safety: std::mutex on all Kortex calls               │
│  Resource mgmt: unique_ptr RAII on all SDK objects            │
│  Error model:   bool returns + exceptions for unrecoverable  │
└──────────────────────────┬───────────────────────────────────┘
                           │  protobuf commands via TCP/IP
                           ▼
┌──────────────────────────────────────────────────────────────┐
│                     Kortex SDK Layer                         │
│  TransportClientTcp → RouterClient → SessionManager          │
│  → BaseClient (motion, gripper, state, safety)               │
└──────────────────────────┬───────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────────┐
│              Kinova Gen3 7-DOF + Robotiq 2F-85               │
│              (TCP/IP, default 192.168.1.10:10000)            │
└──────────────────────────────────────────────────────────────┘
```

## Key Design Decisions

| Decision | Choice | Why |
|----------|--------|-----|
| Resource management | `unique_ptr` RAII, reverse destruction order | Zero leaks, exception-safe, no manual `delete` |
| Thread safety | Single `std::mutex` on all Kortex calls | SDK is not thread-safe; one lock is simple and correct |
| E-stop | `std::atomic<bool>` set before mutex lock | Instant visibility to all threads, even if mutex is held |
| Sync + Async | `std::future<bool>` via `std::async` | Sync for simple use, async for non-blocking motion |
| Units | Radians (public), degrees (internal) | Robotics convention externally, Kortex convention internally |
| Gripper polling | 100ms loop with 5s timeout | `SendGripperCommand` is non-blocking; must poll completion |
| Object detection | Commanded vs actual position gap | No dedicated sensor; Robotiq stall = position gap |
| Testability | `#ifdef USE_KORTEX_MOCK` with mock SDK | Full test suite runs without hardware, CI/CD compatible |

## Dependencies

- g++ with C++17 support
- CMake 3.16+
- Google Test (`sudo apt install libgtest-dev cmake`)

## Project Structure

```
wrapper/
├── include/kinova_interface/
│   ├── KinovaInterface.hpp       # Class declaration, public API
│   └── Pose.hpp                  # Cartesian pose struct
├── mock/
│   └── kortex_mock.hpp           # Mock SDK (structs, stubs, stall simulation)
├── src/
│   └── KinovaInterface.cpp       # Implementation (~800 lines)
├── tests/
│   ├── test_connection.cpp       # 4 tests: connect, disconnect, invalid IP, no-op
│   ├── test_motion.cpp           # 7 tests: motion, state reading, safety
│   └── gripper_test.cpp          # 14 tests: position, open/close, object detection
└── CMakeLists.txt
```

## Build

```bash
mkdir -p build && cd build
cmake .. -DUSE_KORTEX_MOCK=ON
make -j$(nproc)
```

## Run Tests

```bash
./test_connection        # 4 tests
./test_motion            # 7 tests
./test_gripper           # 14 tests

# Filter specific suite
./test_gripper --gtest_filter="GripperTest.*"
```

**25 tests passing (mock SDK).** Real hardware validation on scheduled lab visits.

## API

```cpp
// Connection
bool connect(const std::string& ip, uint32_t port = 10000);
void disconnect();
bool isConnected() const;

// Motion (angles in radians)
bool moveToJointAngles(const std::vector<double>& angles);
std::future<bool> moveToJointAnglesAsync(const std::vector<double>& angles);
bool moveToCartesianPose(const Pose& pose);
std::future<bool> moveToCartesianPoseAsync(const Pose& pose);

// Gripper (Robotiq 2F-85)
bool openGripper(double speed = 0.1);
bool closeGripper(double force = 40.0, double speed = 0.1);
bool setGripperPosition(double position, double speed = 0.1);  // 0.0=open, 1.0=closed
double getGripperPosition();                                     // returns -1.0 on failure
bool isObjectDetected();                                         // stall-based detection

// State
std::vector<double> getJointAngles();
Pose getCurrentPose();
std::vector<double> getWrench();  // [Fx, Fy, Fz, Tx, Ty, Tz]

// Safety
void emergencyStop();
bool isEStopActive() const;
bool clearEmergencyStop();
bool setSpeedLimit(double fraction);  // 0.0 to 1.0
```

## ROS2 Integration

The wrapper is consumed by `KinovaLifecycleController`, a ROS2 lifecycle node providing:
- Joint state publishing at 10 Hz (`/joint_states`)
- `MoveToPose` action server (`/kinova/move_to_pose`) with feedback and cancellation
- Controlled startup/shutdown via lifecycle transitions (configure → activate → deactivate → cleanup)
- Component-based architecture loaded into a shared container for efficient resource usage

## Hardware

| Component | Model | Interface |
|-----------|-------|-----------|
| Arm | Kinova Gen3 7-DOF | TCP/IP via Kortex SDK |
| Gripper | Robotiq 2F-85 | Via Kortex `SendGripperCommand` (internal interconnect) |
| Camera | Intel RealSense D435i | Wrist-mounted, ROS2 integration Week 5 |
| OS | Ubuntu 24.04 | ROS2 Jazzy |

## Roadmap

- [x] Connection management (RAII, auto-cleanup, reconnection)
- [x] Joint and Cartesian motion (sync + async)
- [x] Emergency stop + joint limit validation
- [x] Gripper control (position, open/close, object detection)
- [x] ROS2 lifecycle node + action server + component architecture
- [x] 25 Google Tests (connection, motion, gripper)
- [ ] Trajectory execution — multi-waypoint with progress callback (Week 2)
- [ ] Cartesian velocity control (Week 3)
- [ ] Force/torque threshold callbacks (Week 4)

## Known Issues

**`bad_function_call` on activation (Kortex SDK):** A single message prints to stderr after connecting to the real Gen3. Originates from a closed-source SDK internal thread. No functional impact — joint data publishes, all transitions succeed. Cannot be patched.
