/**
 * @file test_hw_fk_ik.cpp
 * @brief Hardware validation of custom FK/IK against Kortex API on real Kinova Gen3.
 *
 * Tests:
 *   1. FK accuracy: compare computeFK() output vs Kortex GetMeasuredCartesianPose()
 *   2. IK convergence: verify solveIK() reaches the real robot's current pose
 *   3. IK vs Kortex: confirm full chain (IK -> FK -> tool offset) matches hardware
 *   4. Orientation (D-14 DH-rotation gate): geodesic angle between our FK rotation
 *      and Kortex's reported orientation. Tool-offset-independent (pure rotation),
 *      so it isolates the DH twist chain from all translation bookkeeping.
 *
 * Expected results:
 *   - FK vs Kortex position error < ~1 cm (nominal-vs-calibrated DH residual)
 *   - IK vs FK error < 0.1 mm (internal solver convergence)
 *   - Orientation geodesic error: sub-degree across ALL poses under the CORRECT
 *     Euler convention -> DH rotation chain confirmed -> D-14 closes.
 */

#include "kinova_wrapper/KinovaInterface.hpp"
#include <kinova_kinematics/KinovaKinematics.hpp>
#include <iostream>
#include <cmath>
#include <algorithm>        // std::clamp
#include <Eigen/Geometry>   // AngleAxisd

// -----------------------------------------------------------------------------
// Geodesic angle between two rotations, in degrees.
//   R_err = R_fk^T * R_kortex ; theta = arccos((trace(R_err) - 1)/2)
//   trace(AB) = trace(BA) => order of the product does not change the ANGLE,
//   only the residual axis frame. Clamp guards acos against FP drift past +/-1.
// This is an OBSERVATION-side check: R_fk feeds the 6D orientation obs; R_kortex
// is the oracle. Not used in the 1 kHz loop.
// -----------------------------------------------------------------------------
static double geodesicDeg(const Eigen::Matrix3d& R_fk,
                          const Eigen::Matrix3d& R_kortex)
{
    Eigen::Matrix3d R_err = R_fk.transpose() * R_kortex;
    double c = (R_err.trace() - 1.0) / 2.0;
    c = std::clamp(c, -1.0, 1.0);
    return std::acos(c) * 180.0 / M_PI;
}

int main() {
    using namespace kinova_wrapper;

    KinovaKinematics kinematics;
    KinovaInterface kinova;

    // --- Connect to real robot ---
    bool ok = kinova.connect("192.168.1.10");
    std::cout << "Connect(): " << (ok ? "SUCCESS" : "FAILED") << std::endl;
    if (!ok) return 1;
    std::cout << "====================" << std::endl;

    // --- Read real joint angles (radians) from Kortex ---
    auto joint_angles = kinova.getJointAngles();

    std::cout << "Joint angles (rad): ";
    for (const auto& v : joint_angles) std::cout << v << " ";
    std::cout << std::endl;

    // --- Read Kortex Cartesian pose (measured at Tool Center Point) ---
    auto kortex_pose = kinova.getCurrentPose();
    std::cout << "Kortex TCP position: "
              << kortex_pose.x << " " << kortex_pose.y << " " << kortex_pose.z << std::endl;

    // --- Compute FK using our DH chain + tool offset ---
    std::array<double, 7> q{};
    for (int i = 0; i < 7; i++) q[i] = joint_angles[i];

    // --- FK vs Kortex at TWO tool offsets (D-14 before/after, free via ctor) ---
    //   0.12  : matches Kortex's configured transform -> clean DH gate (Diagnostic A)
    //   0.113 : our real physical tool -> ships in production
    auto fk_vs_kortex = [&](double offset, const char* label) -> Eigen::Vector3d {
        KinovaKinematics k(offset);
        Eigen::Vector3d p = k.getPosition(k.computeFK(q));
        double dx = kortex_pose.x - p(0);
        double dy = kortex_pose.y - p(1);
        double dz = kortex_pose.z - p(2);
        std::cout << label << " off=" << offset
                  << "  dxyz=[" << dx << ", " << dy << ", " << dz << "]"
                  << "  |e|=" << std::sqrt(dx*dx + dy*dy + dz*dz) << " m\n";
        return {dx, dy, dz};
    };

    Eigen::Vector3d e012 = fk_vs_kortex(0.12,  "FK(0.12)  vs Kortex:");  // Diagnostic A
    Eigen::Vector3d e113 = fk_vs_kortex(0.113, "FK(0.113) vs Kortex:");
    Eigen::Vector3d d = e113 - e012;                                      // Diagnostic B
    std::cout << "delta(0.113-0.12) dxyz=[" << d(0) << ", " << d(1) << ", " << d(2)
              << "]  |d|=" << d.norm() << " m  (expect 0.007 on tool-z)\n";
    std::cout << "===============================\n";

    // production FK (0.113) for the orientation + IK sections below
    auto T = kinematics.computeFK(q);
    auto fk_pos = kinematics.getPosition(T);

    // -------------------------------------------------------------------------
    // ORIENTATION GATE (D-14): geodesic angle, FK rotation vs Kortex orientation.
    //
    // Kortex exposes orientation ONLY as three Euler/Tait-Bryan angles
    // (theta_x/y/z, degrees) [SPEC: Kortex User Guide, per Sahil 24 Aug]. The
    // SEQUENCE (intrinsic vs extrinsic) is [UNVERIFIED] -- would be confirmed by
    // the Kortex Pose message definition in the API reference. So we build
    // R_kortex under BOTH common XYZ conventions and print both. The correct one
    // is whichever is consistently sub-degree across ALL poses; a wrong sequence
    // may fluke small at one pose but diverges as the angles grow.
    // -------------------------------------------------------------------------
    Eigen::Matrix3d R_fk = kinematics.getRotation(T);

    const double d2r = M_PI / 180.0;
    const double ax = kortex_pose.theta_x * d2r;
    const double ay = kortex_pose.theta_y * d2r;
    const double az = kortex_pose.theta_z * d2r;

    const Eigen::AngleAxisd Rx(ax, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd Ry(ay, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd Rz(az, Eigen::Vector3d::UnitZ());

    // Candidate A: extrinsic XYZ (rotate about FIXED base axes x->y->z) = Rz*Ry*Rx
    const Eigen::Matrix3d R_kortex_extrinsic = (Rz * Ry * Rx).toRotationMatrix();
    // Candidate B: intrinsic XYZ (rotate about MOVING axes x->y->z)     = Rx*Ry*Rz
    const Eigen::Matrix3d R_kortex_intrinsic = (Rx * Ry * Rz).toRotationMatrix();

    std::cout << "Orientation geodesic error:\n"
              << "   extrinsic XYZ (Rz*Ry*Rx): "
              << geodesicDeg(R_fk, R_kortex_extrinsic) << " deg\n"
              << "   intrinsic XYZ (Rx*Ry*Rz): "
              << geodesicDeg(R_fk, R_kortex_intrinsic) << " deg\n";
    std::cout << "===============================\n";

    // --- IK: solve for joint angles that reach the current real pose ---
    std::cout << "IK testing..." << std::endl;

    std::array<double, 7> zero_guess{};
    auto ik_result = kinematics.solveIK(T, zero_guess);
    auto T_check = kinematics.computeFK(ik_result.joint_states);
    auto ik_pos = kinematics.getPosition(T_check);

    // IK vs FK: proves solver convergence (should be < 0.1mm)
    double dx = ik_pos(0) - fk_pos(0);
    double dy = ik_pos(1) - fk_pos(1);
    double dz = ik_pos(2) - fk_pos(2);
    double ik_internal_error = std::sqrt(dx * dx + dy * dy + dz * dz);
    std::cout << "IK vs FK error:      " << ik_internal_error << " m" << std::endl;

    // IK vs Kortex: validates full chain against real hardware
    dx = ik_pos(0) - kortex_pose.x;
    dy = ik_pos(1) - kortex_pose.y;
    dz = ik_pos(2) - kortex_pose.z;
    double ik_kortex_error = std::sqrt(dx * dx + dy * dy + dz * dz);
    std::cout << "IK vs Kortex error:  " << ik_kortex_error << " m" << std::endl;
    std::cout << "===============================" << std::endl;

    kinova.disconnect();
    return 0;
}
