#pragma once

#include "DRFLEx.h"

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace doosan
{

class DoosanController
{
public:
    static constexpr int kNumJoints = 6;
    using JointArray = std::array<float, kNumJoints>;

    enum class RobotSystem
    {
        Real,
        Virtual,
    };

    struct Config
    {
        std::string ip = "192.168.137.100";
        unsigned int port = 12345;
        std::string rt_ip;
        unsigned int rt_port = 12347;
        RobotSystem system = RobotSystem::Real;
        float rt_period_sec = 0.001f;
        int rt_loss_count = 4;
        float command_time_sec = 0.001f;
        float position_move_time_sec = 8.0f;
        JointArray rt_velocity_limit = {70.f, 70.f, 70.f, 70.f, 70.f, 70.f};
        JointArray rt_acceleration_limit = {70.f, 70.f, 70.f, 70.f, 70.f, 70.f};
    };

    DoosanController();
    ~DoosanController();

    DoosanController(const DoosanController &) = delete;
    DoosanController &operator=(const DoosanController &) = delete;

    bool initialize(
        const Config &config,
        bool enable_realtime_control = true,
        std::chrono::seconds timeout = std::chrono::seconds(15));

    bool positionControl(
        const JointArray &position,
        const JointArray &velocity = JointArray{},
        const JointArray &acceleration = JointArray{},
        float command_time_sec = 0.0f);

    bool positionControl(
        const JointArray &position,
        const JointArray &velocity,
        const JointArray &acceleration,
        float move_time_sec,
        float command_time_sec);

    bool velocityControl(
        const JointArray &velocity,
        const JointArray &acceleration,
        float command_time_sec = 0.0f);

    bool velocityControlToPosition(
        const JointArray &target_position,
        const JointArray &max_velocity,
        const JointArray &acceleration,
        float gain,
        float tolerance_deg,
        float timeout_sec,
        float command_time_sec = 0.0f);

    bool taskTrajectoryCsvPositionControl(
        const std::string &csv_path,
        float command_time_sec = 0.0f,
        float max_joint_step_deg = 20.0f,
        float max_joint_velocity_deg_s = 250.0f,
        float max_joint_acceleration_deg_s2 = 1500.0f);

    bool jointTrajectoryCsvPositionControl(
        const std::string &csv_path,
        float command_time_sec = 0.0f,
        float max_joint_step_deg = 20.0f,
        float max_joint_velocity_deg_s = 250.0f,
        float max_joint_acceleration_deg_s2 = 1500.0f);

    bool jointTrajectoryCsvVelocityControl(
        const std::string &csv_path,
        float command_time_sec = 0.0f,
        float position_gain = 4.0f,
        float max_joint_step_deg = 20.0f,
        float max_joint_velocity_deg_s = 70.0f,
        float max_joint_acceleration_deg_s2 = 500.0f,
        float preposition_max_joint_velocity_deg_s = 20.0f,
        float preposition_max_joint_acceleration_deg_s2 = 150.0f);

    bool taskTrajectoryCsvVelocityControl(
        const std::string &csv_path,
        const std::string &urdf_path,
        const std::string &tcp_frame = "link_6",
        float command_time_sec = 0.0f,
        float position_gain = 8.0f,
        float orientation_gain = 4.0f,
        float damping = 0.03f,
        float max_joint_velocity_deg_s = 70.0f,
        float max_joint_acceleration_deg_s2 = 500.0f,
        float max_joint_step_deg = 20.0f);

    bool connect(const Config &config);
    bool servoOn(std::chrono::seconds timeout = std::chrono::seconds(15));
    bool servoOff();

    bool startRealtimeControl();
    void stopRealtimeControl();

    void setJointPositionTarget(
        const JointArray &position,
        const JointArray &velocity = JointArray{},
        const JointArray &acceleration = JointArray{},
        float command_time_sec = 0.0f);

    void setJointVelocityTarget(
        const JointArray &velocity,
        const JointArray &acceleration,
        float command_time_sec = 0.0f);

    void holdCommand();
    void shutdown();

    bool isConnected() const;
    bool isServoOn() const;
    bool isRealtimeControlRunning() const;

    // Read-only telemetry for stationary scan capture (deg, deg/s).
    // Fails if either measurement is unavailable; never substitutes a target/zero.
    bool readActualJointState(JointArray &position, JointArray &velocity);

    static RobotSystem robotSystemFromString(const std::string &value);

    struct TaskTrajectoryPoint
    {
        double time_sec = 0.0;
        std::array<double, 3> position = {};
        std::array<double, 9> rotation = {};
        std::array<double, 3> linear_velocity = {};
        std::array<double, 3> angular_velocity = {};
        std::array<float, kNumJoints> doosan_task_pose = {};
    };

    struct CartesianVelocityContext;

private:
    using Clock = std::chrono::steady_clock;

    enum class CommandMode
    {
        Hold,
        JointPosition,
        JointVelocity,
        JointVelocityTarget,
        JointTrajectory,
        JointVelocityTrajectory,
        TaskVelocityTrajectory,
    };

    struct JointTrajectoryPoint
    {
        double time_sec = 0.0;
        JointArray position = {};
        JointArray velocity = {};
        JointArray acceleration = {};
    };

    struct Command
    {
        CommandMode mode = CommandMode::Hold;
        JointArray position = {};
        JointArray velocity = {};
        JointArray acceleration = {};
        float command_time_sec = 0.015f;
        bool trajectory_active = false;
        bool stop_rt_when_done = false;
        Clock::time_point trajectory_start = {};
        float trajectory_duration_sec = 0.0f;
        Clock::time_point velocity_target_start = {};
        Clock::time_point velocity_target_last_update = {};
        float velocity_gain = 1.0f;
        float velocity_tolerance_deg = 0.5f;
        float velocity_timeout_sec = 20.0f;
        float position_gain = 8.0f;
        float orientation_gain = 4.0f;
        float damping = 0.03f;
        float max_joint_velocity_deg_s = 70.0f;
        float max_joint_acceleration_deg_s2 = 500.0f;
        JointArray velocity_target_command = {};
        JointArray previous_velocity_command = {};
        JointArray velocity_profile_direction = {};
        JointArray velocity_profile_peak = {};
        JointArray velocity_profile_accel_time = {};
        JointArray velocity_profile_cruise_time = {};
        JointArray velocity_profile_total_time = {};
        float velocity_profile_duration_sec = 0.0f;
        JointArray a0 = {};
        JointArray a1 = {};
        JointArray a2 = {};
        JointArray a3 = {};
        JointArray a4 = {};
        JointArray a5 = {};
        JointArray target_position = {};
        std::shared_ptr<const std::vector<JointTrajectoryPoint>> joint_trajectory;
        std::shared_ptr<const std::vector<TaskTrajectoryPoint>> task_trajectory;
        std::shared_ptr<CartesianVelocityContext> cartesian_velocity_context;
    };

    bool waitForControlAccessAndStandby(std::chrono::seconds timeout);
    bool readCurrentJointState(JointArray *position, JointArray *velocity);
    bool holdCurrentPosition();
    bool connectRealtimeControl();
    bool startRealtimeLoop();
    void stopRealtimeSession();
    void disconnectRealtimeControl();
    bool setJointPositionTrajectory(
        const JointArray &position,
        const JointArray &final_velocity,
        const JointArray &final_acceleration,
        float move_time_sec,
        float command_time_sec);
    bool setJointVelocityPositionTarget(
        const JointArray &target_position,
        const JointArray &max_velocity,
        const JointArray &acceleration,
        float gain,
        float tolerance_deg,
        float timeout_sec,
        float command_time_sec);
    bool setTaskTrajectoryCsvPositionTarget(
        const std::string &csv_path,
        float command_time_sec,
        float max_joint_step_deg,
        float max_joint_velocity_deg_s,
        float max_joint_acceleration_deg_s2);
    bool setJointTrajectoryPositionTarget(
        std::shared_ptr<const std::vector<JointTrajectoryPoint>> trajectory,
        float command_time_sec);
    bool setJointTrajectoryVelocityTarget(
        std::shared_ptr<const std::vector<JointTrajectoryPoint>> trajectory,
        float command_time_sec,
        float position_gain,
        float max_joint_velocity_deg_s,
        float max_joint_acceleration_deg_s2);
    bool loadTaskTrajectoryCsv(
        const std::string &csv_path,
        float max_joint_step_deg,
        float max_joint_velocity_deg_s,
        float max_joint_acceleration_deg_s2,
        std::vector<JointTrajectoryPoint> *trajectory);
    bool loadJointTrajectoryCsv(
        const std::string &csv_path,
        float max_joint_step_deg,
        float max_joint_velocity_deg_s,
        float max_joint_acceleration_deg_s2,
        std::vector<JointTrajectoryPoint> *trajectory);
    bool loadCartesianTaskTrajectoryCsv(
        const std::string &csv_path,
        std::vector<TaskTrajectoryPoint> *trajectory);
    bool setTaskVelocityTrajectoryTarget(
        std::shared_ptr<const std::vector<TaskTrajectoryPoint>> trajectory,
        std::shared_ptr<CartesianVelocityContext> context,
        float command_time_sec,
        float position_gain,
        float orientation_gain,
        float damping,
        float max_joint_velocity_deg_s,
        float max_joint_acceleration_deg_s2);
    bool solveClosestIk(
        const std::array<float, kNumJoints> &task_pose,
        const JointArray &reference_joint,
        JointArray *joint,
        int *solution_space);
    void realtimeLoop();

    void handleAccessControl(MONITORING_ACCESS_CONTROL access);
    void handleRobotState(ROBOT_STATE state);
    void handleDisconnected();

    static void onAccessControl(MONITORING_ACCESS_CONTROL access);
    static void onRobotState(ROBOT_STATE state);
    static void onDisconnected();

    static const char *robotStateToString(ROBOT_STATE state);
    static const char *accessControlToString(MONITORING_ACCESS_CONTROL access);
    static const char *robotSystemToString(RobotSystem system);

    DRAFramework::CDRFLEx drfl_;
    Config config_;

    mutable std::mutex drfl_mutex_;
    std::mutex command_mutex_;
    std::thread realtime_thread_;

    std::atomic_bool connected_{false};
    std::atomic_bool servo_on_{false};
    std::atomic_bool realtime_control_connected_{false};
    std::atomic_bool realtime_control_running_{false};
    std::atomic_bool realtime_loop_running_{false};
    std::atomic_bool has_control_access_{false};
    std::atomic_bool is_standby_{false};
    std::atomic_bool disconnected_{false};

    Command command_;

    static DoosanController *active_instance_;
};

}  // namespace doosan
