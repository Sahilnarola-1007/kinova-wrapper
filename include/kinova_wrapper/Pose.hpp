#pragma once

namespace kinova_wrapper {

struct Pose {
    double x = 0.0;        // meters
    double y = 0.0;        // meters
    double z = 0.0;        // meters
    double theta_x = 0.0;  // degrees (Kortex convention)
    double theta_y = 0.0;  // degrees
    double theta_z = 0.0;  // degrees
};

}  // namespace kinova_wrapper
