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

```mermaid
graph TD
    A["<b>User Code / ROS2</b><br/>Lifecycle nodes, action servers,<br/>pick-and-place logic"] 
    -->|"single-line calls"| B

    subgraph B["KinovaInterface Wrapper"]
        direction TB
        B1["<b>Connection</b><br/>connect() · disconnect()<br/>isConnected()"]
        B2["<b>Motion</b><br/>moveToJointAngles()<br/>moveToCartesianPose()<br/>async variants"]
        B3["<b>Gripper</b><br/>open/close/setPosition<br/>isObjectDetected()"]
        B4["<b>State</b><br/>getJointAngles()<br/>getCurrentPose()<br/>getWrench()"]
        B5["<b>Safety</b><br/>emergencyStop()<br/>clearEStop()<br/>setSpeedLimit()"]
        B6["<b>Trajectory</b><br/>executeTrajectory()<br/>async + feedback callback"]
    end

    B -->|"protobuf over TCP/IP"| C["<b>Kortex SDK</b><br/>TransportClientTcp → RouterClient<br/>→ SessionManager → BaseClient"]
    C -->|"TCP/IP"| D["<b>Kinova Gen3 7-DOF</b><br/>+ Robotiq 2F-85<br/>192.168.1.10:10000"]

    style A fill:#4a90d9,color:#fff,stroke:#2c5f8a
    style B fill:#f0f4f8,stroke:#2c5f8a,stroke-width:2px
    style B1 fill:#fff,stroke:#4a90d9
    style B2 fill:#fff,stroke:#4a90d9
    style B3 fill:#fff,stroke:#4a90d9
    style B4 fill:#fff,stroke:#4a90d9
    style B5 fill:#fff,stroke:#4a90d9
    style B6 fill:#fff,stroke:#4a90d9
    style C fill:#6c757d,color:#fff,stroke:#495057
    style D fill:#28a745,color:#fff,stroke:#1e7e34
```

## Key Design Decisions

```mermaid
graph LR
    subgraph Resource["🔒 Resource Management"]
        R1["unique_ptr RAII"] --> R2["Reverse destruction order<br/>base → session → router → transport"]
    end

    subgraph Thread["🧵 Thread Safety"]
        T1["Single std::mutex"] --> T2["All Kortex calls protected"]
        T3["std::atomic flags"] --> T4["E-stop visible instantly<br/>even if mutex is held"]
    end

    subgraph Motion["⚡ Motion Control"]
        M1["Sync: bool return"] --> M2["Simple blocking calls"]
        M3["Async: std::future"] --> M4["Non-blocking motion"]
    end

    subgraph Gripper["🤖 Gripper Control"]
        G1["SendGripperCommand"] --> G2["Non-blocking → poll loop<br/>100ms interval, 5s timeout"]
        G3["Object detection"] --> G4["Commanded vs actual gap<br/>Stall = object grasped"]
    end

    subgraph Testing["🧪 Testability"]
        TE1["#ifdef USE_KORTEX_MOCK"] --> TE2["33 mock tests without hardware<br/>6 hardware test suites on real Gen3"]
    end

    style Resource fill:#e8f4fd,stroke:#2196F3,stroke-width:2px
    style Thread fill:#fff3e0,stroke:#FF9800,stroke-width:2px
    style Motion fill:#e8f5e9,stroke:#4CAF50,stroke-width:2px
    style Gripper fill:#fce4ec,stroke:#E91E63,stroke-width:2px
    style Testing fill:#f3e5f5,stroke:#9C27B0,stroke-width:2px
```

> **Units convention:** Radians in public API, degrees internally (Kortex convention). Conversion happens at the API boundary — callers never deal with degrees.

## Dependencies

- g++ with C++17 support
- CMake 3.16+
- Google Test (`sudo apt install libgtest-dev cmake`)

## Project Structure

```
wrapper/
├── include/kinova_interface/
│   ├── KinovaInterface.hpp
│   └── Pose.hpp
├── mock/
│   └── kortex_mock.hpp
├── src/
│   └── KinovaInterface.cpp
├── tests/
│   ├── test_connection.cpp       # 4 tests (mock)
│   ├── test_motion.cpp           # 7 tests (mock)
│   ├── gripper_test.cpp          # 14 tests (mock)
│   ├── TrajectoryTest.cpp        # 8 tests (mock)
│   └── hardware/
│       ├── test_hw_connection.cpp # Real robot connect/disconnect
│       ├── test_hw_motion.cpp     # Joint motion on real arm
│       ├── test_hw_state.cpp      # State reading validation
│       ├── test_hw_gripper.cpp    # Robotiq open/close/detect
│       ├── test_hw_estop.cpp      # E-stop mid-motion
│       └── test_hw_fk_ik.cpp      # FK/IK vs Kortex validation
└── CMakeLists.txt
```

## Build

```bash
mkdir -p build && cd build
cmake .. -DUSE_KORTEX_MOCK=ON
make -j$(nproc)
```

## Run Tests

### Mock Tests (no hardware required)
./test_connection        # 4 tests
./test_motion            # 7 tests
./test_gripper           # 14 tests
./test_trajectory        # 8 tests

### Hardware Tests (real Gen3 required)
./hw_test_connection     # Connect/disconnect
./hw_test_motion         # Small joint motions
./hw_test_state          # Joint angle/pose reading
./hw_test_gripper        # Robotiq 2F-85 open/close/object detection
./hw_test_estop          # E-stop interrupts motion
./hw_test_fk_ik          # FK/IK validated against Kortex API

**33 mock tests passing. 6 hardware test suites validated on real Gen3.**



## API

```cpp

// Types
struct TrajectoryPoint {
    std::vector<double> joint_angles;  // radians
    double time_from_start;            // seconds
};

using MotionCallback = std::function<void(
    const std::vector<double>& current_joints, double progress)>;
    
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

// Trajectory
bool executeTrajectory(const std::vector<TrajectoryPoint>& waypoints,
                       MotionCallback feedback_cb = nullptr);
std::future<bool> executeTrajectoryAsync(const std::vector<TrajectoryPoint>& waypoints,
                                          MotionCallback feedback_cb = nullptr);
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

## Roadmap

- [x] Connection management (RAII, auto-cleanup, reconnection)
- [x] Joint and Cartesian motion (sync + async)
- [x] Emergency stop + joint limit validation
- [x] Gripper control (position, open/close, object detection)
- [x] Trajectory execution — multi-waypoint with progress callback
- [x] ROS2 lifecycle node + action server + component architecture
- [x] 33 Google Tests (connection, motion, gripper, trajectory)
- [x] Hardware validated — 6 test suites on real Gen3
- [x] FK/IK validated against Kortex API (error < 6mm)
- [ ] Cartesian velocity control (Week 3)
- [ ] Force/torque threshold callbacks (Week 4)
## Known Issues

**`bad_function_call` on activation (Kortex SDK):** A single message prints to stderr after connecting to the real Gen3. Originates from a closed-source SDK internal thread. No functional impact — joint data publishes, all transitions succeed. Cannot be patched.
