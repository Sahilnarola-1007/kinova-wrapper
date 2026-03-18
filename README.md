# KinovaInterface

A production-grade C++ wrapper around the Kinova Kortex API for the Gen3 7-DOF robotic arm.

## Why This Exists

The raw Kortex API requires 50+ lines of boilerplate just to connect, uses manual memory
management with raw pointers, and has no real error handling. Every project ends up
copy-pasting the same setup code.

This wrapper reduces all of that to clean, safe, single-line calls:

```cpp
// Without wrapper: ~60 lines of boilerplate
// With wrapper:
KinovaInterface kinova;
kinova.connect("192.168.1.10");
kinova.moveToJointAngles({0, 0.26, 0, -1.05, 0, -0.78, 0});
// destructor auto-cleans everything
```

## Features

- RAII resource management — no manual cleanup, no memory leaks
- Thread-safe via `std::mutex` protecting all Kortex calls
- Emergency stop with atomic flag — visible to all threads instantly
- Joint limits queried from robot at connect time (not hardcoded)
- Sync and async motion commands (`std::future<bool>`)
- Clean error handling — `bool` returns for expected failures

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
│   └── kortex_mock.hpp       # Mock SDK for testing without hardware
├── src/
│   └── KinovaInterface.cpp
├── tests/
│   ├── test_connection.cpp   # 4 tests
│   └── test_motion.cpp       # 7 tests
└── CMakeLists.txt
```

## Build

```bash
mkdir -p build && cd build
cmake ..
make test_connection test_motion
```

## Run Tests

```bash
./test_connection    # 4 tests: connect, disconnect, invalid IP, safe no-op
./test_motion        # 7 tests: motion commands, state reading, safety
```

## API Overview

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

// State
std::vector<double> getJointAngles();
Pose getCurrentPose();
std::vector<double> getWrench();  // [Fx, Fy, Fz, Tx, Ty, Tz]

// Safety
void emergencyStop();
bool clearEmergencyStop();
bool setSpeedLimit(double fraction);  // 0.0 to 1.0
```

## Hardware

Tested on Kinova Gen3 7-DOF over TCP/IP (Ubuntu 24.04).
Mock header (`kortex_mock.hpp`) allows full compilation and testing without hardware.
