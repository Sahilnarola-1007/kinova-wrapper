#pragma once

// =============================================================================
// Mock Kortex API Types
// =============================================================================
// These stubs mirror the real Kortex SDK object hierarchy:
//   TransportClientTcp → RouterClient → SessionManager → BaseClient
//   TransportClientUdp → RouterClient → SessionManager → BaseCyclicClient
//
// Purpose: Let us compile and test wrapper + 1 kHz loop logic without the real SDK.
// On the lab machine, USE_KORTEX_MOCK=OFF pulls in the real headers instead.
// =============================================================================
//
// CYCLIC LAYER — added 30 Sep 2026 to unblock Step 1.
//
// Why it exists: the Step 1 definition of done is a 60 s MOCK run with zero
// missed deadlines. Before this addition the mock had no BaseCyclic namespace
// at all, so nothing cyclic could be built without the arm attached. That also
// breaks the "a hiring engineer can clone and run it" property.
//
// SERVO MODEL — read this before trusting any number that comes out of it.
//
//   q_meas[k] = q_meas[k-1] + K * (q_cmd - q_meas[k-1]) * dt
//
// K defaults to 63.8 /s. That figure is [INFERRED] from hardware — a
// through-origin least-squares fit on joint 7 only, n=3, spread ~1.5 % — and it
// is NOT a documented Kortex control law. It reproduces both the measured-anchor
// failure and the commanded-anchor success, which is why it is the best model
// available, not why it is correct.
//
//   *** The mock is a TEST HARNESS, never EVIDENCE. No number produced here
//   *** may be reported as a property of the real servo.
//
// Deliberately NOT modelled: the ~104-cycle startup transient observed on
// hardware. Its cause is unverified, and modelling an unexplained artifact
// would manufacture confidence we have not earned.
//
// Deliberately NOT modelled: the firmware watchdog. A clean 60 s mock run says
// nothing about whether the real arm would have faulted.
//
// FOUR OPEN QUESTIONS ARE WIRED IN AS SWITCHES
//
// All four are pending a reply from Kinova support (sent 30 Sep 2026). Each
// default is the PESSIMISTIC choice — the one that forces the loop to defend
// itself:
//
//   set_enforce_joint_limits(bool)   default FALSE
//       Does the firmware enforce joint limits on the low-level cyclic path, or
//       is the host solely responsible? Default false means the mock enforces
//       nothing, so SafetyFilter must carry the whole burden. If Kinova confirms
//       firmware enforcement, flip it to true and confirm the loop still behaves.
//
//   set_wrap_positions(bool)         default TRUE
//       Continuous joints (1,3,5,7) report position wrapped to [0,360).
//       [MEASURED — Web App]; cyclic-API path not yet confirmed.
//       Set false only to prove the loop also survives an unwrapped convention.
//
//   set_feedback_pre_step(bool)      default TRUE          [ADDED 1 Oct 2026]
//       Kinova support question 4: does the Feedback returned by Refresh()
//       reflect the arm state BEFORE the command in that same call was applied,
//       or AFTER? Default TRUE = pre-step = the pessimistic reading: the caller's
//       FK, and therefore R_base_task, is built from a pose that is one cycle
//       stale. If the loop is correct under pre-step it is correct under
//       post-step; the reverse does not hold.
//       RESOLVE against the measured q_send-vs-q_meas phase lag in
//       sinusoid_tracking.csv before trusting either setting on hardware.
//
//   set_check_command_id(bool)       default FALSE         [ADDED 1 Oct 2026]
//       The real arm uses frame_id and per-actuator command_id to detect stale
//       or duplicated packets. The mock cannot reproduce the firmware's reaction,
//       but it CAN catch the host-side bug: a loop that forgets to bump the ids
//       every cycle. Off by default so existing tests are unaffected; turn it ON
//       in the Step 1 test, where a missed bump is a real defect.
//
// Run the 60 s loop under BOTH settings of each switch. Surviving every
// combination is what makes the loop correct before the answers arrive, rather
// than correct by luck afterwards.
//
// TEST ISOLATION
//
// detail::ArmState is a PROCESS-WIDE SINGLETON. reset() clears positions,
// velocities and the servoing flag but deliberately does NOT touch the switches
// or the servo gain — a test that sets a switch and then calls reset() keeps its
// switch, which is almost always what was meant. To get full isolation, declare
// a mock_control::ScopedReset at the top of the test: it saves every knob,
// resets the arm, and restores the knobs on scope exit.
// =============================================================================

#include <string>
#include <vector>
#include <stdexcept>
#include <functional>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <algorithm>

namespace k_api {

// =============================================================================
// Error type — the real RouterClient takes an error callback taking KError
// =============================================================================
class KError {
public:
    KError() = default;
    explicit KError(std::string msg) : msg_(std::move(msg)) {}
    std::string toString() const { return msg_; }
private:
    std::string msg_;
};

class KDetailedException : public std::runtime_error {
public:
    explicit KDetailedException(const std::string& what)
        : std::runtime_error(what) {}
};

// =============================================================================
// Transport Layer
// =============================================================================
// Common base so RouterClient can hold either transport, mirroring the real
// SDK where RouterClient takes an ITransportClient*.
class ITransportClient {
public:
    virtual ~ITransportClient() = default;
    virtual void connect(const std::string& ip, uint32_t port) = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() const = 0;
};

class TransportClientTcp : public ITransportClient {
public:
    void connect(const std::string& ip, uint32_t port) override {
        (void)port;
        if (ip.empty()) throw std::runtime_error("Invalid IP address");
        connected_ = true;
    }
    void disconnect() override { connected_ = false; }
    bool isConnected() const override { return connected_; }
private:
    bool connected_ = false;
};

// Real-time cyclic path (UDP port 10001 on hardware).
class TransportClientUdp : public ITransportClient {
public:
    void connect(const std::string& ip, uint32_t port) override {
        (void)port;
        if (ip.empty()) throw std::runtime_error("Invalid IP address");
        connected_ = true;
    }
    void disconnect() override { connected_ = false; }
    bool isConnected() const override { return connected_; }
private:
    bool connected_ = false;
};

// =============================================================================
// Router Layer
// =============================================================================
// The real RouterClient takes (transport*, error_callback). The error_callback
// parameter is defaulted here so existing single-argument mock call sites in
// KinovaInterface.cpp keep compiling unchanged.
class RouterClient {
public:
    using ErrorCallback = std::function<void(KError)>;

    explicit RouterClient(ITransportClient* transport,
                          ErrorCallback on_error = nullptr)
        : transport_(transport), on_error_(std::move(on_error)) {}

    void SetActivationStatus(bool active) { active_ = active; }
    bool isActive() const { return active_; }

private:
    ITransportClient* transport_;   // non-owning
    ErrorCallback     on_error_;
    bool              active_ = true;
};

// =============================================================================
// Session Layer
// =============================================================================
// Legacy plain-struct form, kept so existing call sites still build.
struct CreateSessionInfo {
    std::string username;
    std::string password;
    uint32_t session_timeout_ms = 60000;
};

// Real-SDK-shaped form: k_api::Session::CreateSessionInfo with setters.
namespace Session {

class CreateSessionInfo {
public:
    void set_username(const std::string& u) { username_ = u; }
    void set_password(const std::string& p) { password_ = p; }
    void set_session_inactivity_timeout(uint32_t ms)    { session_timeout_ms_ = ms; }
    void set_connection_inactivity_timeout(uint32_t ms) { connection_timeout_ms_ = ms; }

    const std::string& username() const { return username_; }
    const std::string& password() const { return password_; }
    uint32_t session_inactivity_timeout() const    { return session_timeout_ms_; }
    uint32_t connection_inactivity_timeout() const { return connection_timeout_ms_; }

private:
    std::string username_;
    std::string password_;
    uint32_t    session_timeout_ms_    = 0;
    uint32_t    connection_timeout_ms_ = 0;
};

}  // namespace Session

class SessionManager {
public:
    explicit SessionManager(RouterClient* router) : router_(router) {}

    // Legacy overload.
    void CreateSession(const CreateSessionInfo& info) {
        if (info.username.empty() || info.password.empty())
            throw std::runtime_error("Authentication failed");
        session_active_ = true;
    }

    // Real-SDK-shaped overload.
    // P2 rule: BOTH timeouts must be set or real sessions die silently. The mock
    // enforces it so the failure surfaces here rather than on hardware.
    void CreateSession(const Session::CreateSessionInfo& info) {
        if (info.username().empty() || info.password().empty())
            throw std::runtime_error("Authentication failed");
        if (info.session_inactivity_timeout() == 0 ||
            info.connection_inactivity_timeout() == 0) {
            throw std::runtime_error(
                "Both session_inactivity_timeout and connection_inactivity_timeout "
                "must be set (P2) — a real session would die silently");
        }
        session_active_ = true;
    }

    void CloseSession() { session_active_ = false; }
    bool isSessionActive() const { return session_active_; }

private:
    RouterClient* router_;  // non-owning
    bool session_active_ = false;
};

// =============================================================================
// Shared simulated arm state
// =============================================================================
// BaseClient and BaseCyclicClient both observe one arm, so the joint state must
// be shared rather than duplicated per client.
namespace detail {

inline constexpr uint32_t kNumJoints = 7;

// Verified values from design_decisions.md (degrees).
// Joints 1,3,5,7 are continuous rotation — no position limit.
// Joints 2,4,6 are symmetrically limited.
//
// D-27 WARNING: this is one of SEVERAL copies of this table in the codebase
// (KinovaKinematics.cpp, SafetyFilter.hpp, safety_filter.md, KinovaInterface.hpp,
// and here). The manifest fix must collapse all of them. Note that
// KinovaInterface.hpp currently marks joint 4 as unlimited, which disagrees with
// this table and with the verified radian values — resolve before trusting either.
struct JointLimitDeg { bool continuous; double abs_limit; };

inline const JointLimitDeg kJointLimits[kNumJoints] = {
    {true,    0.0},   // J1 continuous
    {false, 128.97},  // J2  ±2.2515 rad
    {true,    0.0},   // J3 continuous
    {false, 147.82},  // J4  ±2.5800 rad
    {true,    0.0},   // J5 continuous
    {false, 120.30},  // J6  ±2.0996 rad
    {true,    0.0}    // J7 continuous
};

class ArmState {
public:
    static ArmState& instance() {
        static ArmState s;
        return s;
    }

    // --- Servo model parameters ---
    void   set_servo_gain(double k)  { servo_gain_ = k; }
    double servo_gain() const        { return servo_gain_; }

    // Open-question switches — see the file header.
    void set_enforce_joint_limits(bool v) { enforce_limits_ = v; }
    bool enforce_joint_limits() const     { return enforce_limits_; }

    void set_wrap_positions(bool v) { wrap_positions_ = v; }
    bool wrap_positions() const     { return wrap_positions_; }

    // [ADDED 1 Oct 2026] Kinova support question 4. See the file header.
    void set_feedback_pre_step(bool v) { feedback_pre_step_ = v; }
    bool feedback_pre_step() const     { return feedback_pre_step_; }

    // [ADDED 1 Oct 2026] Host-side staleness check. See the file header.
    void set_check_command_id(bool v) { check_command_id_ = v; }
    bool check_command_id() const     { return check_command_id_; }

    // Force a fixed integration step instead of measured wall time. Useful for
    // deterministic unit tests; leave off for loop runs so scheduler behaviour
    // shows up in the result.
    void   set_fixed_dt(double dt_s) { fixed_dt_s_ = dt_s; }
    void   clear_fixed_dt()          { fixed_dt_s_ = 0.0; }
    double fixed_dt() const          { return fixed_dt_s_; }

    void set_servoing_low_level(bool v) { low_level_ = v; }
    bool servoing_low_level() const     { return low_level_; }

    // --- Packet-id bookkeeping for set_check_command_id ---
    // Kept here rather than in BaseCyclicClient so that reset() clears it: a new
    // run must not inherit the previous run's last-seen ids.
    bool     have_last_ids() const             { return have_last_ids_; }
    uint32_t last_frame_id() const             { return last_frame_id_; }
    uint32_t last_command_id(uint32_t i) const { return last_cmd_id_[i]; }

    void note_ids(uint32_t frame_id, const uint32_t* cmd_ids) {
        last_frame_id_ = frame_id;
        for (uint32_t i = 0; i < kNumJoints; ++i) last_cmd_id_[i] = cmd_ids[i];
        have_last_ids_ = true;
    }

    // Clears arm MOTION state. Deliberately does NOT clear the switches or the
    // servo gain — see "TEST ISOLATION" in the file header. Use
    // mock_control::ScopedReset when you need the knobs restored too.
    void reset() {
        for (auto& q : q_true_) q = 0.0;
        for (auto& v : v_true_) v = 0.0;
        for (auto& c : last_cmd_id_) c = 0;
        have_last_tick_ = false;
        have_last_ids_  = false;
        last_frame_id_  = 0;
        low_level_      = false;
    }

    // Advance the first-order servo one step toward the commanded positions.
    // cmd_deg must have kNumJoints entries.
    void step(const double* cmd_deg) {
        const double dt = next_dt();
        for (uint32_t i = 0; i < kNumJoints; ++i) {
            double target = cmd_deg[i];
            if (enforce_limits_ && !kJointLimits[i].continuous) {
                const double lim = kJointLimits[i].abs_limit;
                target = std::clamp(target, -lim, lim);
            }
            double err = target - q_true_[i];
            if (kJointLimits[i].continuous) {
                err = std::fmod(err + 180.0, 360.0);
                if (err < 0.0) err += 360.0;
                err -= 180.0;                      // shortest path around the circle
            }
            const double dq  = servo_gain_ * err * dt;
            q_true_[i] += dq;
            v_true_[i]  = (dt > 0.0) ? dq / dt : 0.0;
        }
    }

    // Reported position — this is where the wrap question is simulated.
    double reported_position(uint32_t i) const {
        const double q = q_true_[i];
        if (!wrap_positions_ || !kJointLimits[i].continuous) return q;
        double w = std::fmod(q, 360.0);
        if (w < 0.0) w += 360.0;
        return w;
    }

    double velocity(uint32_t i) const   { return v_true_[i]; }
    double true_position(uint32_t i) const { return q_true_[i]; }

    void seed_position(uint32_t i, double deg) { q_true_[i] = deg; }

private:
    ArmState() { reset(); }

    double next_dt() {
        if (fixed_dt_s_ > 0.0) return fixed_dt_s_;
        const auto now = std::chrono::steady_clock::now();
        if (!have_last_tick_) {
            last_tick_      = now;
            have_last_tick_ = true;
            return 0.0;   // first call advances nothing
        }
        const double dt =
            std::chrono::duration<double>(now - last_tick_).count();
        last_tick_ = now;
        // Clamp so a breakpoint or a stalled scheduler cannot teleport the arm.
        return std::clamp(dt, 0.0, 0.05);
    }

    double q_true_[kNumJoints]{};
    double v_true_[kNumJoints]{};

    // [INFERRED] from hardware, joint 7 only, n=3. NOT a Kortex spec.
    double servo_gain_ = 63.8;   // [1/s]

    bool   enforce_limits_    = false;
    bool   wrap_positions_    = true;   // continuous joints wrap to [0,360) by default
    bool   feedback_pre_step_ = true;   // pessimistic: feedback is one cycle stale
    bool   check_command_id_  = false;  // off unless a test asks for it
    bool   low_level_         = false;
    double fixed_dt_s_        = 0.0;

    uint32_t last_frame_id_ = 0;
    uint32_t last_cmd_id_[kNumJoints]{};
    bool     have_last_ids_ = false;

    std::chrono::steady_clock::time_point last_tick_{};
    bool have_last_tick_ = false;
};

}  // namespace detail

// =============================================================================
// Base Client (main control interface)
// =============================================================================
namespace Base {

    // Gripper control modes
    constexpr uint32_t GRIPPER_POSITION = 0;
    constexpr uint32_t GRIPPER_SPEED = 1;
    constexpr uint32_t GRIPPER_FORCE = 2;

    // --- Servoing mode (needed for the low-level cyclic path) ---
    enum class ServoingMode : uint32_t {
        SINGLE_LEVEL_SERVOING = 0,
        LOW_LEVEL_SERVOING    = 1
    };

    class ServoingModeInformation {
    public:
        void set_servoing_mode(ServoingMode m) { mode_ = m; }
        ServoingMode servoing_mode() const     { return mode_; }
    private:
        ServoingMode mode_ = ServoingMode::SINGLE_LEVEL_SERVOING;
    };

    // Simulated joint angle measurement from robot
    struct JointAngle {
        uint32_t joint_identifier = 0;
        double value = 0.0;  // degrees (Kortex convention)
    };

    struct JointAngles {
        std::vector<JointAngle> joint_angles;
    };

    // Simulated Cartesian pose from robot
    struct CartesianPose {
        double x = 0.0, y = 0.0, z = 0.0;           // meters
        double theta_x = 0.0, theta_y = 0.0, theta_z = 0.0;  // degrees
    };

    // Simulated wrench (force/torque) from F/T sensor
    struct Wrench {
        double force_x = 0.0, force_y = 0.0, force_z = 0.0;
        double torque_x = 0.0, torque_y = 0.0, torque_z = 0.0;
    };

    // Simulated joint limit info
    struct JointLimitInfo {
        double min_value = 0.0;  // degrees
        double max_value = 0.0;  // degrees
    };

    // Simulated action for motion commands
    struct Action {
        JointAngles target_joint_angles;
        CartesianPose target_pose;
        bool is_joint_action = false;
        bool is_cartesian_action = false;
    };

    // Gripper finger — single actuator data
    struct Finger {
        uint32_t finger_identifier = 0;
        double value = 0.0;
    };

    // Gripper — contains list of fingers
    struct Gripper {
        std::vector<Finger> finger;
    };

    // Gripper command — what you send
    struct GripperCommand {
        uint32_t mode = 0;
        Gripper gripper;
    };

    // Gripper request — what you ask for when reading
    struct GripperRequest {
        uint32_t mode = 0;
    };

    //Pure twist command
    struct Twist
        {
            double linear_x=0.0, linear_y=0.0, linear_z=0.0;
            double angular_x=0.0,angular_y=0.0,angular_z=0.0;
        };


    // Complete Twist command with nested twist and reference frame
    struct TwistCommand{
                uint32_t reference_frame = 0;
                Twist twist;
        };

    class BaseClient {
    public:
        explicit BaseClient(RouterClient* router)
            : router_(router) {
            // Real robot always has joint positions — initialize with 7 zeros
            for (uint32_t i = 0; i < 7; ++i) {
                JointAngle ja;
                ja.joint_identifier = i;
                ja.value = 0.0;
                current_joint_angles_.joint_angles.push_back(ja);
            }
        }

        // --- Motion ---
        void ExecuteAction(const Action& action) {
            // Real SDK: sends protobuf action to robot, blocks until done
            if (e_stop_) {
                throw std::runtime_error("Emergency stop is active");
            }
            // Simulate: store last commanded values
            if (action.is_joint_action) {
                current_joint_angles_ = action.target_joint_angles;
            }
        }

        // --- State Reading ---
        JointAngles GetMeasuredJointAngles() {
            return current_joint_angles_;
        }

        CartesianPose GetMeasuredCartesianPose() {
            return current_pose_;
        }

        Wrench GetMeasuredWrench() {
            return current_wrench_;
        }

        // --- Joint Limits ---
        std::vector<JointLimitInfo> GetJointLimits() {
            // Values mirror detail::kJointLimits — see the D-27 warning there.
            // Continuous joints are reported as ±360 for backward compatibility
            // with existing call sites; that is a THIRD sentinel convention in
            // this codebase and the manifest fix must eliminate it.
            std::vector<JointLimitInfo> out;
            out.reserve(detail::kNumJoints);
            for (uint32_t i = 0; i < detail::kNumJoints; ++i) {
                const auto& L = detail::kJointLimits[i];
                const double lim = L.continuous ? 360.0 : L.abs_limit;
                out.push_back({-lim, lim});
            }
            return out;
        }

        // --- Servoing mode ---
        void SetServoingMode(const ServoingModeInformation& info) {
            detail::ArmState::instance().set_servoing_low_level(
                info.servoing_mode() == ServoingMode::LOW_LEVEL_SERVOING);
        }

        ServoingModeInformation GetServoingMode() const {
            ServoingModeInformation info;
            info.set_servoing_mode(
                detail::ArmState::instance().servoing_low_level()
                    ? ServoingMode::LOW_LEVEL_SERVOING
                    : ServoingMode::SINGLE_LEVEL_SERVOING);
            return info;
        }

        // --- Safety ---
        void ApplyEmergencyStop() {
            e_stop_ = true;
        }

        void ClearFaults() {
            e_stop_ = false;
        }

        // Gripper methods
        void setSimulateObject(bool val) { simulate_object_ = val; }

        void SendGripperCommand(const GripperCommand& cmd) {
            if (!cmd.gripper.finger.empty()) {
                stored_gripper_position_ = cmd.gripper.finger[0].value;
            }
        }

        Gripper GetMeasuredGripperMovement(const GripperRequest& req) {
            (void)req;
            Gripper g;
            Finger f;
            f.finger_identifier = 1;

            // Simulate object stall: gripper can't fully close
            if (simulate_object_ && stored_gripper_position_ > 0.5) {
                f.value = 0.4;  // stalled at 0.4 instead of reaching 1.0
            } else {
                f.value = stored_gripper_position_;  // instant arrival
            }

            g.finger.push_back(f);
            return g;
        }

        //---Velocity commands----
        void SendTwistCommand(const TwistCommand& T){
            if(e_stop_){
                throw std::runtime_error("Emergency stop is active");
            }

            stored_twist_command=T;

        }

        void Stop(){
            stored_twist_command.twist.linear_x=0.0;
            stored_twist_command.twist.linear_y=0.0;
            stored_twist_command.twist.linear_z=0.0;
            stored_twist_command.twist.angular_x=0.0;
            stored_twist_command.twist.angular_y=0.0;
            stored_twist_command.twist.angular_z=0.0;
        }

        TwistCommand GetLastTwist(){
               return stored_twist_command;
        }


    private:
        RouterClient* router_;  // non-owning
        bool e_stop_ = false;
        JointAngles current_joint_angles_;
        CartesianPose current_pose_;
        Wrench current_wrench_;
        double stored_gripper_position_ = 0.0; // tracks position
        bool simulate_object_ = false;
        TwistCommand stored_twist_command;
    };

}  // namespace Base

// =============================================================================
// BaseCyclic — the 1 kHz real-time path
// =============================================================================
// Mirrors the protobuf-generated accessor style of the real SDK: set_x()/x(),
// add_actuators(), mutable_actuators(i), actuators(i).
namespace BaseCyclic {

class ActuatorCommand {
public:
    void set_command_id(uint32_t id) { command_id_ = id; }
    uint32_t command_id() const      { return command_id_; }

    void set_position(double deg) { position_ = deg; }
    double position() const       { return position_; }

    // Present for interface parity. D-12 closed by scope: the 1 kHz loop and the
    // D-13 sweep command POSITION only, so this field is never set by our code.
    // D-28 (locked, hardware-measured): velocity mode coasts on packet loss with
    // no fault raised — position mode stops because the target stops advancing.
    void set_velocity(double deg_per_s) { velocity_ = deg_per_s; }
    double velocity() const             { return velocity_; }

private:
    uint32_t command_id_ = 0;
    double   position_   = 0.0;
    double   velocity_   = 0.0;
};

class ActuatorFeedback {
public:
    double position() const { return position_; }
    double velocity() const { return velocity_; }
    double torque()   const { return torque_; }

    void set_position(double v) { position_ = v; }
    void set_velocity(double v) { velocity_ = v; }
    void set_torque(double v)   { torque_ = v; }

private:
    double position_ = 0.0;
    double velocity_ = 0.0;
    double torque_   = 0.0;
};

class Command {
public:
    void set_frame_id(uint32_t id) { frame_id_ = id; }
    uint32_t frame_id() const      { return frame_id_; }

    ActuatorCommand* add_actuators() {
        actuators_.emplace_back();
        return &actuators_.back();
    }

    ActuatorCommand* mutable_actuators(int i) { return &actuators_.at(static_cast<size_t>(i)); }
    const ActuatorCommand& actuators(int i) const { return actuators_.at(static_cast<size_t>(i)); }
    int actuators_size() const { return static_cast<int>(actuators_.size()); }

private:
    uint32_t                     frame_id_ = 0;
    std::vector<ActuatorCommand> actuators_;
};

class Feedback {
public:
    const ActuatorFeedback& actuators(int i) const {
        return actuators_.at(static_cast<size_t>(i));
    }
    ActuatorFeedback* mutable_actuators(int i) {
        return &actuators_.at(static_cast<size_t>(i));
    }
    int actuators_size() const { return static_cast<int>(actuators_.size()); }

    void resize(size_t n) { actuators_.resize(n); }

private:
    std::vector<ActuatorFeedback> actuators_;
};

class BaseCyclicClient {
public:
    explicit BaseCyclicClient(RouterClient* router) : router_(router) {}

    // Read-only: does not advance the servo model.
    Feedback RefreshFeedback() {
        return snapshot();
    }

    // The cyclic call. Advances the first-order servo one step using the
    // commanded positions, then returns the arm state.
    //
    // WHICH state is returned depends on set_feedback_pre_step():
    //   true  (default) — the state BEFORE this cycle's command was applied.
    //                     The caller's FK is then one cycle stale.
    //   false           — the state AFTER this cycle's command was applied.
    //
    // Which one the real firmware does is Kinova support question 4, still open.
    // The default is the pessimistic reading; a loop that is correct under it is
    // correct under either.
    //
    // NOTE: the one-cycle command pipeline in low_level_feedback_loop.cpp lives
    // in the CALLER (it stages the command and sends it on the NEXT Refresh).
    // That is a separate lag from this switch and is preserved unchanged.
    Feedback Refresh(const Command& command) {
        auto& arm = detail::ArmState::instance();

        const int n = command.actuators_size();

        // --- Optional host-side staleness check ---
        // Catches the loop forgetting to bump frame_id / command_id each cycle.
        // The real firmware's reaction to a stale packet is NOT modelled here;
        // this only fails fast on a defect that is ours, not the arm's.
        if (arm.check_command_id()) {
            uint32_t ids[detail::kNumJoints];
            for (uint32_t i = 0; i < detail::kNumJoints; ++i) {
                ids[i] = (static_cast<int>(i) < n)
                           ? command.actuators(static_cast<int>(i)).command_id()
                           : 0u;
            }
            if (arm.have_last_ids()) {
                if (command.frame_id() == arm.last_frame_id()) {
                    throw KDetailedException(
                        "mock: frame_id did not advance between Refresh() calls "
                        "— the loop is sending a stale packet");
                }
                for (uint32_t i = 0; i < detail::kNumJoints; ++i) {
                    if (static_cast<int>(i) < n &&
                        ids[i] == arm.last_command_id(i)) {
                        throw KDetailedException(
                            "mock: actuator command_id did not advance on joint "
                            + std::to_string(i) +
                            " — the loop is sending a stale packet");
                    }
                }
            }
            arm.note_ids(command.frame_id(), ids);
        }

        double cmd[detail::kNumJoints];
        for (uint32_t i = 0; i < detail::kNumJoints; ++i) {
            cmd[i] = (static_cast<int>(i) < n)
                       ? command.actuators(static_cast<int>(i)).position()
                       : arm.true_position(i);   // unaddressed joints hold
        }

        if (arm.feedback_pre_step()) {
            Feedback fb = snapshot();   // state BEFORE this command
            arm.step(cmd);
            return fb;
        }

        arm.step(cmd);
        return snapshot();              // state AFTER this command
    }

private:
    Feedback snapshot() const {
        auto& arm = detail::ArmState::instance();
        Feedback fb;
        fb.resize(detail::kNumJoints);
        for (uint32_t i = 0; i < detail::kNumJoints; ++i) {
            auto* a = fb.mutable_actuators(static_cast<int>(i));
            a->set_position(arm.reported_position(i));
            a->set_velocity(arm.velocity(i));
            a->set_torque(0.0);   // no torque model — Gen3 has no joint-torque control in our stack
        }
        return fb;
    }

    RouterClient* router_;  // non-owning
};

}  // namespace BaseCyclic

// =============================================================================
// Test-only controls
// =============================================================================
// Not part of the real SDK. Guarded by name so a stray call under
// USE_KORTEX_MOCK=OFF fails to compile rather than silently doing nothing.
namespace mock_control {

inline void reset()                        { detail::ArmState::instance().reset(); }
inline void set_servo_gain(double k)       { detail::ArmState::instance().set_servo_gain(k); }
inline void set_enforce_joint_limits(bool v){ detail::ArmState::instance().set_enforce_joint_limits(v); }
inline void set_wrap_positions(bool v)     { detail::ArmState::instance().set_wrap_positions(v); }
inline void set_feedback_pre_step(bool v)  { detail::ArmState::instance().set_feedback_pre_step(v); }
inline void set_check_command_id(bool v)   { detail::ArmState::instance().set_check_command_id(v); }
inline void set_fixed_dt(double dt_s)      { detail::ArmState::instance().set_fixed_dt(dt_s); }
inline void clear_fixed_dt()               { detail::ArmState::instance().clear_fixed_dt(); }
inline void seed_position(uint32_t i, double deg) {
    detail::ArmState::instance().seed_position(i, deg);
}
inline double true_position(uint32_t i) {
    return detail::ArmState::instance().true_position(i);
}

// -----------------------------------------------------------------------------
// ScopedReset — RAII isolation for the process-wide ArmState singleton.
// -----------------------------------------------------------------------------
// Declare one at the top of any test that touches the mock:
//
//     TEST(Foo, Bar) {
//         k_api::mock_control::ScopedReset guard;
//         k_api::mock_control::set_fixed_dt(0.001);
//         ...
//     }   // every knob restored, arm motion state cleared
//
// Without it, gtest's shared process means a switch set by one test silently
// changes the meaning of the next one, and the failure appears in whichever
// test happens to run second. Non-copyable and non-movable on purpose: two
// guards restoring the same singleton would fight.
class ScopedReset {
public:
    ScopedReset()
        : servo_gain_(detail::ArmState::instance().servo_gain()),
          enforce_limits_(detail::ArmState::instance().enforce_joint_limits()),
          wrap_positions_(detail::ArmState::instance().wrap_positions()),
          feedback_pre_step_(detail::ArmState::instance().feedback_pre_step()),
          check_command_id_(detail::ArmState::instance().check_command_id()),
          fixed_dt_s_(detail::ArmState::instance().fixed_dt()) {
        detail::ArmState::instance().reset();
    }

    ~ScopedReset() {
        auto& arm = detail::ArmState::instance();
        arm.reset();
        arm.set_servo_gain(servo_gain_);
        arm.set_enforce_joint_limits(enforce_limits_);
        arm.set_wrap_positions(wrap_positions_);
        arm.set_feedback_pre_step(feedback_pre_step_);
        arm.set_check_command_id(check_command_id_);
        arm.set_fixed_dt(fixed_dt_s_);
    }

    ScopedReset(const ScopedReset&)            = delete;
    ScopedReset& operator=(const ScopedReset&) = delete;
    ScopedReset(ScopedReset&&)                 = delete;
    ScopedReset& operator=(ScopedReset&&)      = delete;

private:
    double servo_gain_;
    bool   enforce_limits_;
    bool   wrap_positions_;
    bool   feedback_pre_step_;
    bool   check_command_id_;
    double fixed_dt_s_;
};

}  // namespace mock_control

}  // namespace k_api
