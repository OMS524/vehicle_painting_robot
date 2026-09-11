#include "doosan_controller.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <unordered_map>

#ifndef DOOSAN_HAS_PINOCCHIO
#define DOOSAN_HAS_PINOCCHIO 0
#endif

#if DOOSAN_HAS_PINOCCHIO
#include <Eigen/Dense>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#endif

namespace doosan
{

DoosanController *DoosanController::active_instance_ = nullptr;

#if DOOSAN_HAS_PINOCCHIO
struct DoosanController::CartesianVelocityContext
{
    pinocchio::Model model;
    pinocchio::Data data;
    pinocchio::FrameIndex frame_id = 0;
    std::string frame_name;

    explicit CartesianVelocityContext(const pinocchio::Model &pinocchio_model)
        : model(pinocchio_model),
          data(model)
    {
    }
};
#else
struct DoosanController::CartesianVelocityContext
{
};
#endif

namespace
{
constexpr double kPi = 3.14159265358979323846;

float commandTimeOrDefault(float requested, float fallback)
{
    return requested > 0.0f ? requested : fallback;
}

std::string trim(std::string value)
{
    auto begin = value.begin();
    while (begin != value.end() &&
           std::isspace(static_cast<unsigned char>(*begin)))
    {
        ++begin;
    }

    auto end = value.end();
    while (end != begin &&
           std::isspace(static_cast<unsigned char>(*(end - 1))))
    {
        --end;
    }

    return std::string(begin, end);
}

std::vector<std::string> splitCsvLine(const std::string &line)
{
    std::vector<std::string> fields;
    std::string field;
    bool in_quotes = false;

    for (char ch : line)
    {
        if (ch == '"')
        {
            in_quotes = !in_quotes;
            continue;
        }
        if (ch == ',' && !in_quotes)
        {
            fields.push_back(trim(field));
            field.clear();
            continue;
        }
        field.push_back(ch);
    }

    fields.push_back(trim(field));
    return fields;
}

bool parseDouble(const std::vector<std::string> &fields,
                 const std::unordered_map<std::string, std::size_t> &columns,
                 const std::string &name,
                 double *value)
{
    const auto it = columns.find(name);
    if (it == columns.end() || it->second >= fields.size())
    {
        return false;
    }

    try
    {
        std::size_t parsed = 0;
        const double parsed_value = std::stod(fields[it->second], &parsed);
        if (parsed == 0 || !std::isfinite(parsed_value))
        {
            return false;
        }
        *value = parsed_value;
        return true;
    }
    catch (...)
    {
        return false;
    }
}

std::string findFirstColumn(
    const std::unordered_map<std::string, std::size_t> &columns,
    const std::vector<std::string> &names)
{
    for (const auto &name : names)
    {
        if (columns.find(name) != columns.end())
        {
            return name;
        }
    }
    return {};
}

std::vector<std::string> jointPositionColumnNames(int joint_index)
{
    const std::string n = std::to_string(joint_index + 1);
    return {
        "q" + n,
        "q" + n + "_deg",
        "joint_" + n,
        "joint_" + n + "_deg",
        "position_" + n,
        "position_" + n + "_deg"};
}

std::vector<std::string> jointVelocityColumnNames(int joint_index)
{
    const std::string n = std::to_string(joint_index + 1);
    return {
        "qd" + n,
        "qdot" + n,
        "qd" + n + "_deg_s",
        "qdot" + n + "_deg_s",
        "velocity_" + n,
        "velocity_" + n + "_deg_s",
        "joint_" + n + "_velocity",
        "joint_" + n + "_velocity_deg_s"};
}

std::vector<std::string> jointAccelerationColumnNames(int joint_index)
{
    const std::string n = std::to_string(joint_index + 1);
    return {
        "qdd" + n,
        "qddot" + n,
        "qdd" + n + "_deg_s2",
        "qddot" + n + "_deg_s2",
        "acceleration_" + n,
        "acceleration_" + n + "_deg_s2",
        "joint_" + n + "_acceleration",
        "joint_" + n + "_acceleration_deg_s2"};
}

double clampUnit(double value)
{
    return std::clamp(value, -1.0, 1.0);
}

double radToDeg(double rad)
{
    return rad * 180.0 / kPi;
}

double degToRad(double deg)
{
    return deg * kPi / 180.0;
}

double wrapAngleDeg(double value)
{
    while (value > 180.0)
    {
        value -= 360.0;
    }
    while (value < -180.0)
    {
        value += 360.0;
    }
    return value;
}

double absAngleDiffDeg(double a, double b)
{
    return std::fabs(wrapAngleDeg(a - b));
}

std::array<double, 9> quaternionXyzwToRotation(double qx, double qy, double qz, double qw)
{
    const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (norm <= 1.0e-12)
    {
        return {1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0};
    }

    qx /= norm;
    qy /= norm;
    qz /= norm;
    qw /= norm;

    const double xx = qx * qx;
    const double yy = qy * qy;
    const double zz = qz * qz;
    const double xy = qx * qy;
    const double xz = qx * qz;
    const double yz = qy * qz;
    const double wx = qw * qx;
    const double wy = qw * qy;
    const double wz = qw * qz;

    return {
        1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
        2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
        2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)};
}

std::array<float, 6> taskPoseFromCsvPoseMetersAndQuatXyzw(
    double x_m,
    double y_m,
    double z_m,
    double qx,
    double qy,
    double qz,
    double qw)
{
    const auto r = quaternionXyzwToRotation(qx, qy, qz, qw);
    const double beta = std::atan2(
        std::sqrt(r[2] * r[2] + r[5] * r[5]),
        clampUnit(r[8]));

    double alpha = 0.0;
    double gamma = 0.0;
    if (std::sin(beta) > 1.0e-9)
    {
        alpha = std::atan2(r[5], r[2]);
        gamma = std::atan2(r[7], -r[6]);
    }
    else
    {
        alpha = std::atan2(r[3], r[0]);
        gamma = 0.0;
    }

    return {
        static_cast<float>(x_m * 1000.0),
        static_cast<float>(y_m * 1000.0),
        static_cast<float>(z_m * 1000.0),
        static_cast<float>(radToDeg(alpha)),
        static_cast<float>(radToDeg(beta)),
        static_cast<float>(radToDeg(gamma))};
}

double maxAbsJointDelta(const doosan::DoosanController::JointArray &a,
                        const doosan::DoosanController::JointArray &b)
{
    double max_delta = 0.0;
    for (int i = 0; i < doosan::DoosanController::kNumJoints; ++i)
    {
        max_delta = std::max(
            max_delta,
            std::fabs(static_cast<double>(a[i]) - static_cast<double>(b[i])));
    }
    return max_delta;
}

double l2JointDelta(const doosan::DoosanController::JointArray &a,
                    const doosan::DoosanController::JointArray &b)
{
    double sum = 0.0;
    for (int i = 0; i < doosan::DoosanController::kNumJoints; ++i)
    {
        const double delta = static_cast<double>(a[i]) - static_cast<double>(b[i]);
        sum += delta * delta;
    }
    return std::sqrt(sum);
}

doosan::DoosanController::JointArray interpolateJointArray(
    const doosan::DoosanController::JointArray &a,
    const doosan::DoosanController::JointArray &b,
    double alpha)
{
    doosan::DoosanController::JointArray out = {};
    const double u = std::clamp(alpha, 0.0, 1.0);
    for (int i = 0; i < doosan::DoosanController::kNumJoints; ++i)
    {
        out[i] = static_cast<float>(
            (1.0 - u) * static_cast<double>(a[i]) +
            u * static_cast<double>(b[i]));
    }
    return out;
}

std::array<double, 3> interpolateArray3(
    const std::array<double, 3> &a,
    const std::array<double, 3> &b,
    double alpha)
{
    std::array<double, 3> out = {};
    const double u = std::clamp(alpha, 0.0, 1.0);
    for (int i = 0; i < 3; ++i)
    {
        out[i] = (1.0 - u) * a[i] + u * b[i];
    }
    return out;
}

doosan::DoosanController::JointArray clampJointArray(
    const doosan::DoosanController::JointArray &value,
    double limit_abs)
{
    doosan::DoosanController::JointArray out = value;
    if (limit_abs <= 0.0)
    {
        return out;
    }

    for (int i = 0; i < doosan::DoosanController::kNumJoints; ++i)
    {
        out[i] = static_cast<float>(
            std::clamp(static_cast<double>(out[i]), -limit_abs, limit_abs));
    }
    return out;
}

#if DOOSAN_HAS_PINOCCHIO
Eigen::Matrix3d rotationFromRowMajor(const std::array<double, 9> &rotation)
{
    Eigen::Matrix3d out;
    out << rotation[0], rotation[1], rotation[2],
        rotation[3], rotation[4], rotation[5],
        rotation[6], rotation[7], rotation[8];
    return out;
}

std::array<double, 9> rowMajorFromRotation(const Eigen::Matrix3d &rotation)
{
    return {
        rotation(0, 0), rotation(0, 1), rotation(0, 2),
        rotation(1, 0), rotation(1, 1), rotation(1, 2),
        rotation(2, 0), rotation(2, 1), rotation(2, 2)};
}

Eigen::Vector3d vectorFromArray3(const std::array<double, 3> &value)
{
    return Eigen::Vector3d(value[0], value[1], value[2]);
}

Eigen::Quaterniond quaternionFromRowMajor(const std::array<double, 9> &rotation)
{
    Eigen::Quaterniond quat(rotationFromRowMajor(rotation));
    quat.normalize();
    return quat;
}

doosan::DoosanController::TaskTrajectoryPoint interpolateTaskTrajectoryPoint(
    const std::vector<doosan::DoosanController::TaskTrajectoryPoint> &trajectory,
    double elapsed_sec)
{
    if (trajectory.empty())
    {
        return {};
    }

    if (trajectory.size() == 1 || elapsed_sec <= trajectory.front().time_sec)
    {
        return trajectory.front();
    }
    if (elapsed_sec >= trajectory.back().time_sec)
    {
        return trajectory.back();
    }

    const auto upper = std::lower_bound(
        trajectory.begin(),
        trajectory.end(),
        elapsed_sec,
        [](const doosan::DoosanController::TaskTrajectoryPoint &point, double t) {
            return point.time_sec < t;
        });
    if (upper == trajectory.begin())
    {
        return *upper;
    }
    if (upper == trajectory.end())
    {
        return trajectory.back();
    }

    const auto lower = upper - 1;
    const double segment_dt = std::max(upper->time_sec - lower->time_sec, 1.0e-9);
    const double alpha = (elapsed_sec - lower->time_sec) / segment_dt;

    auto out = *lower;
    out.time_sec = elapsed_sec;
    out.position = interpolateArray3(lower->position, upper->position, alpha);
    out.linear_velocity = interpolateArray3(lower->linear_velocity, upper->linear_velocity, alpha);
    out.angular_velocity = interpolateArray3(lower->angular_velocity, upper->angular_velocity, alpha);
    const Eigen::Quaterniond q0 = quaternionFromRowMajor(lower->rotation);
    const Eigen::Quaterniond q1 = quaternionFromRowMajor(upper->rotation);
    out.rotation = rowMajorFromRotation(q0.slerp(std::clamp(alpha, 0.0, 1.0), q1).toRotationMatrix());
    return out;
}
#endif
}  // namespace

DoosanController::DoosanController()
{
    active_instance_ = this;
}

DoosanController::~DoosanController()
{
    shutdown();
    if (active_instance_ == this)
    {
        active_instance_ = nullptr;
    }
}

bool DoosanController::initialize(
    const Config &config,
    bool enable_realtime_control,
    std::chrono::seconds timeout)
{
    if (!connect(config))
    {
        return false;
    }

    if (!servoOn(timeout))
    {
        shutdown();
        return false;
    }

    if (enable_realtime_control && config_.system == RobotSystem::Real)
    {
        if (!connectRealtimeControl())
        {
            shutdown();
            return false;
        }
    }

    return true;
}

bool DoosanController::positionControl(
    const JointArray &position,
    const JointArray &velocity,
    const JointArray &acceleration,
    float command_time_sec)
{
    return positionControl(
        position,
        velocity,
        acceleration,
        config_.position_move_time_sec,
        command_time_sec);
}

bool DoosanController::positionControl(
    const JointArray &position,
    const JointArray &velocity,
    const JointArray &acceleration,
    float move_time_sec,
    float command_time_sec)
{
    if (!servo_on_)
    {
        std::cerr << "[robot] positionControl requires initialize()/servoOn()\n";
        return false;
    }

    if (config_.system != RobotSystem::Real)
    {
        std::cerr << "[robot] positionControl uses servoj_rt and requires real robot mode\n";
        return false;
    }

    const bool started_rt = !realtime_control_running_;
    if (!connectRealtimeControl())
    {
        return false;
    }

    if (!setJointPositionTrajectory(position, velocity, acceleration, move_time_sec, command_time_sec))
    {
        if (started_rt)
        {
            stopRealtimeControl();
        }
        return false;
    }

    if (!startRealtimeLoop())
    {
        stopRealtimeControl();
        return false;
    }

    if (started_rt && !startRealtimeControl())
    {
        realtime_loop_running_ = false;
        if (realtime_thread_.joinable())
        {
            realtime_thread_.join();
        }
        stopRealtimeControl();
        return false;
    }

    return true;
}

bool DoosanController::velocityControl(
    const JointArray &velocity,
    const JointArray &acceleration,
    float command_time_sec)
{
    if (!servo_on_)
    {
        std::cerr << "[robot] velocityControl requires initialize()/servoOn()\n";
        return false;
    }

    if (config_.system != RobotSystem::Real)
    {
        std::cerr << "[robot] velocityControl uses speedj_rt and requires real robot mode\n";
        return false;
    }

    const bool started_rt = !realtime_control_running_;
    if (!connectRealtimeControl())
    {
        return false;
    }

    setJointVelocityTarget(velocity, acceleration, command_time_sec);

    if (!startRealtimeLoop())
    {
        stopRealtimeControl();
        return false;
    }

    if (started_rt && !startRealtimeControl())
    {
        realtime_loop_running_ = false;
        if (realtime_thread_.joinable())
        {
            realtime_thread_.join();
        }
        stopRealtimeControl();
        return false;
    }

    return true;
}

bool DoosanController::velocityControlToPosition(
    const JointArray &target_position,
    const JointArray &max_velocity,
    const JointArray &acceleration,
    float gain,
    float tolerance_deg,
    float timeout_sec,
    float command_time_sec)
{
    if (!servo_on_)
    {
        std::cerr << "[robot] velocityControlToPosition requires initialize()/servoOn()\n";
        return false;
    }

    if (config_.system != RobotSystem::Real)
    {
        std::cerr << "[robot] velocityControlToPosition uses speedj_rt and requires real robot mode\n";
        return false;
    }

    const bool started_rt = !realtime_control_running_;
    if (!connectRealtimeControl())
    {
        return false;
    }

    if (!setJointVelocityPositionTarget(
            target_position,
            max_velocity,
            acceleration,
            gain,
            tolerance_deg,
            timeout_sec,
            command_time_sec))
    {
        if (started_rt)
        {
            stopRealtimeControl();
        }
        return false;
    }

    if (!startRealtimeLoop())
    {
        stopRealtimeControl();
        return false;
    }

    if (started_rt && !startRealtimeControl())
    {
        realtime_loop_running_ = false;
        if (realtime_thread_.joinable())
        {
            realtime_thread_.join();
        }
        stopRealtimeControl();
        return false;
    }

    return true;
}

bool DoosanController::taskTrajectoryCsvPositionControl(
    const std::string &csv_path,
    float command_time_sec,
    float max_joint_step_deg,
    float max_joint_velocity_deg_s,
    float max_joint_acceleration_deg_s2)
{
    if (!servo_on_)
    {
        std::cerr << "[robot] taskTrajectoryCsvPositionControl requires initialize()/servoOn()\n";
        return false;
    }

    if (config_.system != RobotSystem::Real)
    {
        std::cerr << "[robot] taskTrajectoryCsvPositionControl uses servoj_rt and requires real robot mode\n";
        return false;
    }

    auto trajectory = std::make_shared<std::vector<JointTrajectoryPoint>>();
    if (!loadTaskTrajectoryCsv(
            csv_path,
            max_joint_step_deg > 0.0f ? max_joint_step_deg : 20.0f,
            max_joint_velocity_deg_s > 0.0f ? max_joint_velocity_deg_s : 250.0f,
            max_joint_acceleration_deg_s2 > 0.0f ? max_joint_acceleration_deg_s2 : 1500.0f,
            trajectory.get()))
    {
        return false;
    }

    JointArray current_position = {};
    if (!readCurrentJointState(&current_position, nullptr))
    {
        std::cerr << "[robot] failed to read current joint state before trajectory pre-position\n";
        return false;
    }

    constexpr double kFirstPointToleranceDeg = 0.1;
    const double first_point_error = maxAbsJointDelta(current_position, trajectory->front().position);
    if (first_point_error > kFirstPointToleranceDeg)
    {
        std::cout << "[robot] move current joint position to trajectory first point first. "
                  << "max_error=" << first_point_error
                  << "deg duration=" << config_.position_move_time_sec << "s\n";

        if (!positionControl(
                trajectory->front().position,
                JointArray{},
                JointArray{},
                config_.position_move_time_sec,
                config_.command_time_sec))
        {
            return false;
        }

        const auto deadline = Clock::now() +
                              std::chrono::duration_cast<Clock::duration>(
                                  std::chrono::duration<double>(
                                      static_cast<double>(config_.position_move_time_sec) + 5.0));
        while (Clock::now() < deadline)
        {
            if (!realtime_control_running_ && !realtime_loop_running_)
            {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (realtime_control_running_ || realtime_loop_running_)
        {
            std::cerr << "[robot] trajectory first-point pre-position timed out\n";
            stopRealtimeControl();
            return false;
        }

        if (realtime_thread_.joinable())
        {
            realtime_thread_.join();
        }

        JointArray after_position = {};
        if (readCurrentJointState(&after_position, nullptr))
        {
            std::cout << "[robot] trajectory first-point pre-position done max_error="
                      << maxAbsJointDelta(after_position, trajectory->front().position)
                      << "deg\n";
        }
    }

    if (!connectRealtimeControl())
    {
        return false;
    }

    if (!setJointTrajectoryPositionTarget(trajectory, command_time_sec))
    {
        stopRealtimeControl();
        return false;
    }

    if (!startRealtimeLoop())
    {
        stopRealtimeControl();
        return false;
    }

    if (!startRealtimeControl())
    {
        realtime_loop_running_ = false;
        if (realtime_thread_.joinable())
        {
            realtime_thread_.join();
        }
        stopRealtimeControl();
        return false;
    }

    return true;
}

bool DoosanController::jointTrajectoryCsvPositionControl(
    const std::string &csv_path,
    float command_time_sec,
    float max_joint_step_deg,
    float max_joint_velocity_deg_s,
    float max_joint_acceleration_deg_s2)
{
    if (!servo_on_)
    {
        std::cerr << "[robot] jointTrajectoryCsvPositionControl requires initialize()/servoOn()\n";
        return false;
    }

    if (config_.system != RobotSystem::Real)
    {
        std::cerr << "[robot] jointTrajectoryCsvPositionControl uses servoj_rt and requires real robot mode\n";
        return false;
    }

    auto trajectory = std::make_shared<std::vector<JointTrajectoryPoint>>();
    if (!loadJointTrajectoryCsv(
            csv_path,
            max_joint_step_deg > 0.0f ? max_joint_step_deg : 20.0f,
            max_joint_velocity_deg_s > 0.0f ? max_joint_velocity_deg_s : 250.0f,
            max_joint_acceleration_deg_s2 > 0.0f ? max_joint_acceleration_deg_s2 : 1500.0f,
            trajectory.get()))
    {
        return false;
    }

    JointArray current_position = {};
    if (!readCurrentJointState(&current_position, nullptr))
    {
        std::cerr << "[robot] failed to read current joint state before joint trajectory pre-position\n";
        return false;
    }

    constexpr double kFirstPointToleranceDeg = 0.1;
    const double first_point_error = maxAbsJointDelta(current_position, trajectory->front().position);
    if (first_point_error > kFirstPointToleranceDeg)
    {
        std::cout << "[robot] move current joint position to optimized trajectory first point first. "
                  << "max_error=" << first_point_error
                  << "deg duration=" << config_.position_move_time_sec << "s\n";

        if (!positionControl(
                trajectory->front().position,
                JointArray{},
                JointArray{},
                config_.position_move_time_sec,
                config_.command_time_sec))
        {
            return false;
        }

        const auto deadline = Clock::now() +
                              std::chrono::duration_cast<Clock::duration>(
                                  std::chrono::duration<double>(
                                      static_cast<double>(config_.position_move_time_sec) + 5.0));
        while (Clock::now() < deadline)
        {
            if (!realtime_control_running_ && !realtime_loop_running_)
            {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (realtime_control_running_ || realtime_loop_running_)
        {
            std::cerr << "[robot] optimized trajectory first-point pre-position timed out\n";
            stopRealtimeControl();
            return false;
        }

        if (realtime_thread_.joinable())
        {
            realtime_thread_.join();
        }
    }

    if (!connectRealtimeControl())
    {
        return false;
    }

    if (!setJointTrajectoryPositionTarget(trajectory, command_time_sec))
    {
        stopRealtimeControl();
        return false;
    }

    if (!startRealtimeLoop())
    {
        stopRealtimeControl();
        return false;
    }

    if (!startRealtimeControl())
    {
        realtime_loop_running_ = false;
        if (realtime_thread_.joinable())
        {
            realtime_thread_.join();
        }
        stopRealtimeControl();
        return false;
    }

    return true;
}

bool DoosanController::jointTrajectoryCsvVelocityControl(
    const std::string &csv_path,
    float command_time_sec,
    float position_gain,
    float max_joint_step_deg,
    float max_joint_velocity_deg_s,
    float max_joint_acceleration_deg_s2,
    float preposition_max_joint_velocity_deg_s,
    float preposition_max_joint_acceleration_deg_s2)
{
    if (!servo_on_)
    {
        std::cerr << "[robot] jointTrajectoryCsvVelocityControl requires initialize()/servoOn()\n";
        return false;
    }

    if (config_.system != RobotSystem::Real)
    {
        std::cerr << "[robot] jointTrajectoryCsvVelocityControl uses speedj_rt and requires real robot mode\n";
        return false;
    }

    auto trajectory = std::make_shared<std::vector<JointTrajectoryPoint>>();
    if (!loadJointTrajectoryCsv(
            csv_path,
            max_joint_step_deg > 0.0f ? max_joint_step_deg : 20.0f,
            max_joint_velocity_deg_s > 0.0f ? max_joint_velocity_deg_s : 70.0f,
            std::numeric_limits<float>::max(),
            trajectory.get()))
    {
        return false;
    }

    JointArray current_position = {};
    if (!readCurrentJointState(&current_position, nullptr))
    {
        std::cerr << "[robot] failed to read current joint state before joint velocity trajectory pre-position\n";
        return false;
    }

    constexpr double kFirstPointToleranceDeg = 0.1;
    const double first_point_error = maxAbsJointDelta(current_position, trajectory->front().position);
    if (first_point_error > kFirstPointToleranceDeg)
    {
        JointArray preposition_velocity = {};
        JointArray preposition_acceleration = {};
        const float preposition_velocity_limit =
            preposition_max_joint_velocity_deg_s > 0.0f ? preposition_max_joint_velocity_deg_s : 20.0f;
        const float preposition_acceleration_limit =
            preposition_max_joint_acceleration_deg_s2 > 0.0f ? preposition_max_joint_acceleration_deg_s2 : 150.0f;
        preposition_velocity.fill(preposition_velocity_limit);
        preposition_acceleration.fill(preposition_acceleration_limit);

        const float preposition_timeout_sec = std::max(
            15.0f,
            config_.position_move_time_sec + 10.0f);

        std::cout << "[robot] move current joint position to optimized velocity trajectory first point first. "
                  << "max_error=" << first_point_error
                  << "deg v_max=" << preposition_velocity_limit
                  << "deg/s a_max=" << preposition_acceleration_limit
                  << "deg/s^2 timeout=" << preposition_timeout_sec << "s\n";

        if (!velocityControlToPosition(
                trajectory->front().position,
                preposition_velocity,
                preposition_acceleration,
                1.0f,
                static_cast<float>(kFirstPointToleranceDeg),
                preposition_timeout_sec,
                config_.command_time_sec))
        {
            return false;
        }

        const auto deadline = Clock::now() +
                              std::chrono::duration_cast<Clock::duration>(
                                  std::chrono::duration<double>(
                                      static_cast<double>(preposition_timeout_sec) + 5.0));
        while (Clock::now() < deadline)
        {
            if (!realtime_control_running_ && !realtime_loop_running_)
            {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (realtime_control_running_ || realtime_loop_running_)
        {
            std::cerr << "[robot] optimized velocity trajectory first-point pre-position timed out\n";
            stopRealtimeControl();
            return false;
        }

        if (realtime_thread_.joinable())
        {
            realtime_thread_.join();
        }
    }

    if (!connectRealtimeControl())
    {
        return false;
    }

    if (!setJointTrajectoryVelocityTarget(
            trajectory,
            command_time_sec,
            position_gain,
            max_joint_velocity_deg_s,
            max_joint_acceleration_deg_s2))
    {
        stopRealtimeControl();
        return false;
    }

    if (!startRealtimeLoop())
    {
        stopRealtimeControl();
        return false;
    }

    if (!startRealtimeControl())
    {
        realtime_loop_running_ = false;
        if (realtime_thread_.joinable())
        {
            realtime_thread_.join();
        }
        stopRealtimeControl();
        return false;
    }

    return true;
}

bool DoosanController::taskTrajectoryCsvVelocityControl(
    const std::string &csv_path,
    const std::string &urdf_path,
    const std::string &tcp_frame,
    float command_time_sec,
    float position_gain,
    float orientation_gain,
    float damping,
    float max_joint_velocity_deg_s,
    float max_joint_acceleration_deg_s2,
    float max_joint_step_deg)
{
    if (!servo_on_)
    {
        std::cerr << "[robot] taskTrajectoryCsvVelocityControl requires initialize()/servoOn()\n";
        return false;
    }

    if (config_.system != RobotSystem::Real)
    {
        std::cerr << "[robot] taskTrajectoryCsvVelocityControl uses speedj_rt and requires real robot mode\n";
        return false;
    }

#if !DOOSAN_HAS_PINOCCHIO
    (void)csv_path;
    (void)urdf_path;
    (void)tcp_frame;
    (void)command_time_sec;
    (void)position_gain;
    (void)orientation_gain;
    (void)damping;
    (void)max_joint_velocity_deg_s;
    (void)max_joint_acceleration_deg_s2;
    (void)max_joint_step_deg;
    std::cerr << "[robot] task trajectory velocity control requires Pinocchio. "
              << "Install Pinocchio and reconfigure CMake.\n";
    return false;
#else
    auto trajectory = std::make_shared<std::vector<TaskTrajectoryPoint>>();
    if (!loadCartesianTaskTrajectoryCsv(csv_path, trajectory.get()))
    {
        return false;
    }

    pinocchio::Model model;
    try
    {
        pinocchio::urdf::buildModel(urdf_path, model);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[robot] failed to load URDF with Pinocchio: " << e.what() << '\n';
        return false;
    }

    const std::string frame_name = tcp_frame.empty() ? "link_6" : tcp_frame;
    if (!model.existFrame(frame_name))
    {
        std::cerr << "[robot] Pinocchio frame not found: " << frame_name << '\n';
        return false;
    }
    if (model.nv != kNumJoints)
    {
        std::cerr << "[robot] Pinocchio model nv=" << model.nv
                  << " but controller expects " << kNumJoints << " joints\n";
        return false;
    }

    auto context = std::make_shared<CartesianVelocityContext>(model);
    context->frame_id = model.getFrameId(frame_name);
    context->frame_name = frame_name;

    JointArray current_position = {};
    if (!readCurrentJointState(&current_position, nullptr))
    {
        std::cerr << "[robot] failed to read current joint state before velocity trajectory pre-position\n";
        return false;
    }

    JointArray first_joint = {};
    int first_solution = -1;
    if (!solveClosestIk(trajectory->front().doosan_task_pose, current_position, &first_joint, &first_solution))
    {
        std::cerr << "[robot] failed to solve IK for velocity trajectory first point\n";
        return false;
    }

    constexpr double kFirstPointToleranceDeg = 0.1;
    const double first_point_error = maxAbsJointDelta(current_position, first_joint);
    if (first_point_error > kFirstPointToleranceDeg)
    {
        std::cout << "[robot] move current joint position to velocity trajectory first point first. "
                  << "max_error=" << first_point_error
                  << "deg solution=" << first_solution
                  << " duration=" << config_.position_move_time_sec << "s\n";

        if (!positionControl(
                first_joint,
                JointArray{},
                JointArray{},
                config_.position_move_time_sec,
                config_.command_time_sec))
        {
            return false;
        }

        const auto deadline = Clock::now() +
                              std::chrono::duration_cast<Clock::duration>(
                                  std::chrono::duration<double>(
                                      static_cast<double>(config_.position_move_time_sec) + 5.0));
        while (Clock::now() < deadline)
        {
            if (!realtime_control_running_ && !realtime_loop_running_)
            {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (realtime_control_running_ || realtime_loop_running_)
        {
            std::cerr << "[robot] velocity trajectory first-point pre-position timed out\n";
            stopRealtimeControl();
            return false;
        }

        if (realtime_thread_.joinable())
        {
            realtime_thread_.join();
        }
    }

    if (!connectRealtimeControl())
    {
        return false;
    }

    if (!setTaskVelocityTrajectoryTarget(
            trajectory,
            context,
            command_time_sec,
            position_gain,
            orientation_gain,
            damping,
            max_joint_velocity_deg_s,
            max_joint_acceleration_deg_s2))
    {
        stopRealtimeControl();
        return false;
    }

    if (!startRealtimeLoop())
    {
        stopRealtimeControl();
        return false;
    }

    if (!startRealtimeControl())
    {
        realtime_loop_running_ = false;
        if (realtime_thread_.joinable())
        {
            realtime_thread_.join();
        }
        stopRealtimeControl();
        return false;
    }

    std::cout << "[robot] task velocity trajectory started frame=" << frame_name
              << " points=" << trajectory->size()
              << " duration=" << trajectory->back().time_sec
              << "s Kp=" << (position_gain > 0.0f ? position_gain : 8.0f)
              << " Ko=" << (orientation_gain > 0.0f ? orientation_gain : 4.0f)
              << " damping=" << (damping > 0.0f ? damping : 0.03f)
              << '\n';
    return true;
#endif
}

bool DoosanController::connect(const Config &config)
{
    if (connected_)
    {
        return true;
    }

    config_ = config;
    if (config_.rt_ip.empty())
    {
        config_.rt_ip = config_.ip;
    }

    has_control_access_ = false;
    is_standby_ = false;
    disconnected_ = false;
    active_instance_ = this;

    {
        std::lock_guard<std::mutex> lock(drfl_mutex_);
        drfl_.set_on_monitoring_access_control(&DoosanController::onAccessControl);
        drfl_.set_on_monitoring_state(&DoosanController::onRobotState);
        drfl_.set_on_disconnected(&DoosanController::onDisconnected);

        std::cout << "[robot] open_connection ip=" << config_.ip << " port=" << config_.port << '\n';
        if (!drfl_.open_connection(config_.ip, config_.port))
        {
            std::cerr << "[robot] failed to open DRFL connection\n";
            return false;
        }

        SYSTEM_VERSION system_version;
        std::memset(&system_version, 0, sizeof(system_version));
        if (drfl_.get_system_version(&system_version))
        {
            std::cout << "[robot] DRCF version: " << system_version._szController << '\n';
            std::cout << "[robot] DRFL version: " << drfl_.get_library_version() << '\n';
        }

        if (!drfl_.setup_monitoring_version(1))
        {
            std::cerr << "[robot] setup_monitoring_version(1) failed\n";
            drfl_.close_connection();
            return false;
        }
    }

    connected_ = true;
    return true;
}

bool DoosanController::servoOn(std::chrono::seconds timeout)
{
    if (!connected_)
    {
        std::cerr << "[robot] servoOn requested before connect\n";
        return false;
    }

    if (!waitForControlAccessAndStandby(timeout))
    {
        std::cerr << "[robot] control authority/STANDBY timeout\n";
        return false;
    }

    const auto robot_system =
        config_.system == RobotSystem::Real ? ROBOT_SYSTEM_REAL : ROBOT_SYSTEM_VIRTUAL;

    std::lock_guard<std::mutex> lock(drfl_mutex_);
    std::cout << "[robot] set_robot_mode AUTONOMOUS\n";
    if (!drfl_.set_robot_mode(ROBOT_MODE_AUTONOMOUS))
    {
        std::cerr << "[robot] failed to set ROBOT_MODE_AUTONOMOUS\n";
        return false;
    }

    std::cout << "[robot] set_robot_system " << robotSystemToString(config_.system) << '\n';
    if (!drfl_.set_robot_system(robot_system))
    {
        std::cerr << "[robot] failed to set robot system\n";
        return false;
    }

    drfl_.set_auto_servo_off(false, 5.0f);
    drfl_.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);

    servo_on_ = true;
    return true;
}

bool DoosanController::servoOff()
{
    if (!connected_)
    {
        return true;
    }

    stopRealtimeControl();

    std::lock_guard<std::mutex> lock(drfl_mutex_);
    std::cout << "[robot] servo_off\n";
    drfl_.servo_off(STOP_TYPE_QUICK);
    servo_on_ = false;
    return true;
}

bool DoosanController::connectRealtimeControl()
{
    if (!connected_ || !servo_on_)
    {
        std::cerr << "[robot] connectRealtimeControl requires connect() and servoOn()\n";
        return false;
    }

    if (config_.system != RobotSystem::Real)
    {
        std::cerr << "[robot] realtime control is only enabled for real robot mode\n";
        return false;
    }

    if (realtime_control_connected_)
    {
        return true;
    }

    std::lock_guard<std::mutex> lock(drfl_mutex_);
    std::cout << "[robot] connect_rt_control ip=" << config_.rt_ip
              << " port=" << config_.rt_port << '\n';
    if (!drfl_.connect_rt_control(config_.rt_ip, config_.rt_port))
    {
        std::cerr << "[robot] failed to connect RT control\n";
        return false;
    }
    realtime_control_connected_ = true;

    if (!drfl_.set_rt_control_output("v1.0", config_.rt_period_sec, config_.rt_loss_count))
    {
        std::cerr << "[robot] failed to set RT control output\n";
        return false;
    }

    auto velocity_limit = config_.rt_velocity_limit;
    auto acceleration_limit = config_.rt_acceleration_limit;
    if (!drfl_.set_velj_rt(velocity_limit.data()) ||
        !drfl_.set_accj_rt(acceleration_limit.data()))
    {
        std::cerr << "[robot] failed to set RT joint velocity/acceleration limits\n";
        return false;
    }

    return true;
}

bool DoosanController::startRealtimeControl()
{
    if (!connected_ || !servo_on_)
    {
        std::cerr << "[robot] startRealtimeControl requires connect() and servoOn()\n";
        return false;
    }

    if (config_.system != RobotSystem::Real)
    {
        std::cerr << "[robot] realtime control is only enabled for real robot mode\n";
        return false;
    }

    if (realtime_control_running_)
    {
        return true;
    }

    if (!connectRealtimeControl())
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(drfl_mutex_);
    if (!drfl_.start_rt_control())
    {
        std::cerr << "[robot] failed to start RT control\n";
        return false;
    }

    {
        std::lock_guard<std::mutex> command_lock(command_mutex_);
        if (command_.trajectory_active)
        {
            command_.trajectory_start = Clock::now();
        }
        if (command_.mode == CommandMode::JointVelocityTarget)
        {
            command_.velocity_target_start = Clock::now();
            command_.velocity_target_last_update = command_.velocity_target_start;
        }
    }

    realtime_control_running_ = true;
    return true;
}

void DoosanController::stopRealtimeControl()
{
    realtime_loop_running_ = false;

    if (realtime_thread_.joinable() &&
        realtime_thread_.get_id() != std::this_thread::get_id())
    {
        realtime_thread_.join();
    }

    stopRealtimeSession();
}

bool DoosanController::startRealtimeLoop()
{
    if (!realtime_control_connected_)
    {
        std::cerr << "[robot] startRealtimeLoop requires connectRealtimeControl()\n";
        return false;
    }

    if (realtime_loop_running_)
    {
        return true;
    }

    if (realtime_thread_.joinable())
    {
        realtime_thread_.join();
    }

    realtime_loop_running_ = true;
    try
    {
        realtime_thread_ = std::thread(&DoosanController::realtimeLoop, this);
    }
    catch (const std::exception &e)
    {
        realtime_loop_running_ = false;
        std::cerr << "[robot] failed to create realtime thread: " << e.what() << '\n';
        stopRealtimeControl();
        return false;
    }
    catch (...)
    {
        realtime_loop_running_ = false;
        std::cerr << "[robot] failed to create realtime thread\n";
        stopRealtimeControl();
        return false;
    }
    return true;
}

void DoosanController::stopRealtimeSession()
{
    if (!realtime_control_running_.exchange(false))
    {
        return;
    }

    std::lock_guard<std::mutex> lock(drfl_mutex_);
    drfl_.stop_rt_control();
}

void DoosanController::disconnectRealtimeControl()
{
    stopRealtimeControl();

    if (!realtime_control_connected_.exchange(false))
    {
        return;
    }

    std::lock_guard<std::mutex> lock(drfl_mutex_);
    drfl_.disconnect_rt_control();
}

void DoosanController::setJointPositionTarget(
    const JointArray &position,
    const JointArray &velocity,
    const JointArray &acceleration,
    float command_time_sec)
{
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_.mode = CommandMode::JointPosition;
    command_.position = position;
    command_.velocity = velocity;
    command_.acceleration = acceleration;
    command_.command_time_sec = commandTimeOrDefault(command_time_sec, config_.command_time_sec);
    command_.trajectory_active = false;
    command_.stop_rt_when_done = false;
    command_.target_position = position;
}

void DoosanController::setJointVelocityTarget(
    const JointArray &velocity,
    const JointArray &acceleration,
    float command_time_sec)
{
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_.mode = CommandMode::JointVelocity;
    command_.velocity = velocity;
    command_.acceleration = acceleration;
    command_.command_time_sec = commandTimeOrDefault(command_time_sec, config_.command_time_sec);
    command_.trajectory_active = false;
    command_.stop_rt_when_done = false;
}

void DoosanController::holdCommand()
{
    if (servo_on_ && realtime_control_running_ && config_.system == RobotSystem::Real)
    {
        if (holdCurrentPosition())
        {
            return;
        }

        std::cerr << "[robot] failed to read current joint state for hold command\n";
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    command_.mode = CommandMode::Hold;
}

void DoosanController::shutdown()
{
    if (connected_ && servo_on_)
    {
        servoOff();
    }
    else
    {
        stopRealtimeControl();
    }

    disconnectRealtimeControl();

    if (connected_)
    {
        std::lock_guard<std::mutex> lock(drfl_mutex_);
        drfl_.close_connection();
        connected_ = false;
        servo_on_ = false;
    }
}

bool DoosanController::isConnected() const
{
    return connected_;
}

bool DoosanController::isServoOn() const
{
    return servo_on_;
}

bool DoosanController::isRealtimeControlRunning() const
{
    return realtime_control_running_;
}

bool DoosanController::readActualJointState(JointArray &position, JointArray &velocity)
{
    std::lock_guard<std::mutex> lock(drfl_mutex_);
    if (!connected_ || disconnected_)
    {
        return false;
    }
    // Use controller queries, not the last RT buffer after a completed move.
    const auto *pose = drfl_.get_current_posj();
    if (!pose)
    {
        return false;
    }
    JointArray measured_position = {};
    for (int i = 0; i < kNumJoints; ++i)
    {
        measured_position[i] = pose->_fPosition[i];
    }
    const auto *speed = drfl_.get_current_velj();
    if (!speed)
    {
        return false;
    }
    JointArray measured_velocity = {};
    for (int i = 0; i < kNumJoints; ++i)
    {
        measured_velocity[i] = speed->_fVelocity[i];
        if (!std::isfinite(measured_position[i]) || !std::isfinite(measured_velocity[i]))
        {
            return false;
        }
    }
    position = measured_position;
    velocity = measured_velocity;
    return true;
}

DoosanController::RobotSystem DoosanController::robotSystemFromString(const std::string &value)
{
    if (value == "virtual")
    {
        return RobotSystem::Virtual;
    }
    return RobotSystem::Real;
}

bool DoosanController::waitForControlAccessAndStandby(std::chrono::seconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    while (std::chrono::steady_clock::now() < deadline && !disconnected_)
    {
        if (!has_control_access_)
        {
            std::lock_guard<std::mutex> lock(drfl_mutex_);
            std::cout << "[robot] request control access\n";
            drfl_.manage_access_control(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
        }
        else if (!is_standby_)
        {
            std::lock_guard<std::mutex> lock(drfl_mutex_);
            std::cout << "[robot] servo on\n";
            drfl_.set_robot_control(CONTROL_SERVO_ON);
        }
        else
        {
            return true;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return false;
}

bool DoosanController::readCurrentJointState(JointArray *position, JointArray *velocity)
{
    std::lock_guard<std::mutex> lock(drfl_mutex_);
    if (realtime_control_running_)
    {
        auto *data = drfl_.read_data_rt();
        bool rt_data_valid = data && std::isfinite(data->time_stamp) && data->time_stamp > 0.0;
        if (rt_data_valid && position)
        {
            for (int i = 0; i < kNumJoints; ++i)
            {
                rt_data_valid = rt_data_valid && std::isfinite(data->actual_joint_position[i]);
            }
        }
        if (rt_data_valid && velocity)
        {
            for (int i = 0; i < kNumJoints; ++i)
            {
                rt_data_valid = rt_data_valid && std::isfinite(data->actual_joint_velocity[i]);
            }
        }

        if (rt_data_valid)
        {
            if (position)
            {
                for (int i = 0; i < kNumJoints; ++i)
                {
                    (*position)[i] = data->actual_joint_position[i];
                }
            }

            if (velocity)
            {
                for (int i = 0; i < kNumJoints; ++i)
                {
                    (*velocity)[i] = data->actual_joint_velocity[i];
                }
            }

            return true;
        }
    }

    auto *pose = drfl_.get_current_posj();
    if (!pose)
    {
        pose = drfl_.get_current_pose(ROBOT_SPACE_JOINT);
    }
    if (!pose)
    {
        return false;
    }

    if (position)
    {
        for (int i = 0; i < kNumJoints; ++i)
        {
            (*position)[i] = pose->_fPosition[i];
        }
    }

    if (velocity)
    {
        auto *current_velocity = drfl_.get_current_velj();
        if (current_velocity)
        {
            for (int i = 0; i < kNumJoints; ++i)
            {
                (*velocity)[i] = current_velocity->_fVelocity[i];
            }
        }
        else
        {
            velocity->fill(0.0f);
        }
    }

    return true;
}

bool DoosanController::holdCurrentPosition()
{
    JointArray current_position = {};
    JointArray current_velocity = {};

    for (int attempt = 0; attempt < 50; ++attempt)
    {
        if (readCurrentJointState(&current_position, &current_velocity))
        {
            setJointPositionTarget(current_position, JointArray{}, JointArray{}, config_.command_time_sec);
            std::cout << "[robot] holding current joint position: ";
            for (int i = 0; i < kNumJoints; ++i)
            {
                std::cout << current_position[i] << (i + 1 < kNumJoints ? ", " : "\n");
            }
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return false;
}

bool DoosanController::setJointPositionTrajectory(
    const JointArray &position,
    const JointArray &final_velocity,
    const JointArray &final_acceleration,
    float move_time_sec,
    float command_time_sec)
{
    JointArray start_position = {};
    JointArray start_velocity = {};
    bool got_state = false;
    for (int attempt = 0; attempt < 100; ++attempt)
    {
        if (readCurrentJointState(&start_position, &start_velocity))
        {
            got_state = true;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (!got_state)
    {
        std::cerr << "[robot] failed to read current joint state for position trajectory\n";
        return false;
    }

    const float duration = std::max(
        move_time_sec > 0.0f ? move_time_sec : config_.position_move_time_sec,
        std::max(config_.rt_period_sec, 0.001f));
    const float tt = commandTimeOrDefault(command_time_sec, config_.command_time_sec);

    Command next_command;
    next_command.mode = CommandMode::JointPosition;
    next_command.command_time_sec = tt;
    next_command.trajectory_active = true;
    next_command.stop_rt_when_done = true;
    next_command.trajectory_start = Clock::now();
    next_command.trajectory_duration_sec = duration;
    next_command.position = start_position;
    next_command.velocity = final_velocity;
    next_command.acceleration = final_acceleration;
    next_command.target_position = position;

    const double tf = duration;
    const double tf2 = tf * tf;
    const double tf3 = tf2 * tf;
    const double tf4 = tf3 * tf;
    const double tf5 = tf4 * tf;

    for (int i = 0; i < kNumJoints; ++i)
    {
        const double ps = start_position[i];
        const double vs = start_velocity[i];
        const double as = 0.0;
        const double pf = position[i];
        const double vf = final_velocity[i];
        const double af = final_acceleration[i];

        next_command.a0[i] = static_cast<float>(ps);
        next_command.a1[i] = static_cast<float>(vs);
        next_command.a2[i] = static_cast<float>(as / 2.0);
        next_command.a3[i] = static_cast<float>(
            (20.0 * pf - 20.0 * ps - (8.0 * vf + 12.0 * vs) * tf -
             (3.0 * as - af) * tf2) /
            (2.0 * tf3));
        next_command.a4[i] = static_cast<float>(
            (30.0 * ps - 30.0 * pf + (14.0 * vf + 16.0 * vs) * tf +
             (3.0 * as - 2.0 * af) * tf2) /
            (2.0 * tf4));
        next_command.a5[i] = static_cast<float>(
            (12.0 * pf - 12.0 * ps - (6.0 * vf + 6.0 * vs) * tf -
             (as - af) * tf2) /
            (2.0 * tf5));
    }

    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_ = next_command;
    }

    std::cout << "[robot] position trajectory target: ";
    for (int i = 0; i < kNumJoints; ++i)
    {
        std::cout << position[i] << (i + 1 < kNumJoints ? ", " : "");
    }
    std::cout << " duration=" << duration << "s tt=" << tt << "s\n";
    return true;
}

bool DoosanController::setJointVelocityPositionTarget(
    const JointArray &target_position,
    const JointArray &max_velocity,
    const JointArray &acceleration,
    float gain,
    float tolerance_deg,
    float timeout_sec,
    float command_time_sec)
{
    JointArray current_position = {};
    bool got_state = false;
    for (int attempt = 0; attempt < 100; ++attempt)
    {
        if (readCurrentJointState(&current_position, nullptr))
        {
            got_state = true;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (!got_state)
    {
        std::cerr << "[robot] failed to read current joint state for velocity target control\n";
        return false;
    }

    double initial_max_error = 0.0;
    JointArray profile_direction = {};
    JointArray profile_peak_velocity = {};
    JointArray profile_accel_time = {};
    JointArray profile_cruise_time = {};
    JointArray profile_total_time = {};
    double profile_duration = 0.0;
    const double tolerance = static_cast<double>(tolerance_deg > 0.0f ? tolerance_deg : 0.5f);

    for (int i = 0; i < kNumJoints; ++i)
    {
        const double error =
            static_cast<double>(target_position[i]) -
            static_cast<double>(current_position[i]);
        const double distance = std::fabs(error);
        initial_max_error = std::max(initial_max_error, distance);

        if (distance <= tolerance)
        {
            continue;
        }

        const double max_velocity_abs = std::max(
            static_cast<double>(
                std::fabs(max_velocity[i]) > 0.0f
                    ? std::fabs(max_velocity[i])
                    : std::fabs(config_.rt_velocity_limit[i])),
            1.0e-6);
        const double acceleration_abs = std::max(
            static_cast<double>(
                std::fabs(acceleration[i]) > 0.0f
                    ? std::fabs(acceleration[i])
                    : std::fabs(config_.rt_acceleration_limit[i])),
            1.0e-6);

        const double accel_time_to_max = max_velocity_abs / acceleration_abs;
        const double accel_distance_to_max =
            0.5 * acceleration_abs * accel_time_to_max * accel_time_to_max;

        double accel_time = accel_time_to_max;
        double cruise_time = 0.0;
        double peak_velocity = max_velocity_abs;

        if (distance <= 2.0 * accel_distance_to_max)
        {
            peak_velocity = std::sqrt(distance * acceleration_abs);
            accel_time = peak_velocity / acceleration_abs;
        }
        else
        {
            const double cruise_distance = distance - 2.0 * accel_distance_to_max;
            cruise_time = cruise_distance / peak_velocity;
        }

        profile_direction[i] = static_cast<float>(error >= 0.0 ? 1.0 : -1.0);
        profile_peak_velocity[i] = static_cast<float>(peak_velocity);
        profile_accel_time[i] = static_cast<float>(accel_time);
        profile_cruise_time[i] = static_cast<float>(cruise_time);
        profile_total_time[i] = static_cast<float>(2.0 * accel_time + cruise_time);
        profile_duration = std::max(profile_duration, 2.0 * accel_time + cruise_time);
    }

    if (profile_duration > 0.0)
    {
        for (int i = 0; i < kNumJoints; ++i)
        {
            if (profile_total_time[i] <= 0.0f)
            {
                continue;
            }

            const double distance = std::fabs(
                static_cast<double>(target_position[i]) -
                static_cast<double>(current_position[i]));
            const double max_velocity_abs = std::max(
                static_cast<double>(
                    std::fabs(max_velocity[i]) > 0.0f
                        ? std::fabs(max_velocity[i])
                        : std::fabs(config_.rt_velocity_limit[i])),
                1.0e-6);
            const double acceleration_abs = std::max(
                static_cast<double>(
                    std::fabs(acceleration[i]) > 0.0f
                        ? std::fabs(acceleration[i])
                        : std::fabs(config_.rt_acceleration_limit[i])),
                1.0e-6);

            const double discriminant = std::max(
                0.0,
                profile_duration * profile_duration -
                    4.0 * distance / acceleration_abs);
            double accel_time =
                0.5 * (profile_duration - std::sqrt(discriminant));
            double peak_velocity = acceleration_abs * accel_time;
            if (peak_velocity > max_velocity_abs)
            {
                peak_velocity = max_velocity_abs;
                accel_time = peak_velocity / acceleration_abs;
            }
            const double cruise_time = std::max(0.0, profile_duration - 2.0 * accel_time);

            profile_peak_velocity[i] = static_cast<float>(peak_velocity);
            profile_accel_time[i] = static_cast<float>(accel_time);
            profile_cruise_time[i] = static_cast<float>(cruise_time);
            profile_total_time[i] = static_cast<float>(profile_duration);
        }
    }

    Command next_command;
    next_command.mode = CommandMode::JointVelocityTarget;
    next_command.command_time_sec = commandTimeOrDefault(command_time_sec, config_.command_time_sec);
    next_command.trajectory_active = false;
    next_command.stop_rt_when_done = true;
    next_command.velocity_target_start = Clock::now();
    next_command.velocity_target_last_update = next_command.velocity_target_start;
    next_command.velocity_gain = gain > 0.0f ? gain : 1.0f;
    next_command.velocity_tolerance_deg = tolerance_deg > 0.0f ? tolerance_deg : 0.5f;
    next_command.velocity_timeout_sec = timeout_sec > 0.0f ? timeout_sec : 20.0f;
    next_command.position = current_position;
    next_command.velocity = max_velocity;
    next_command.acceleration = acceleration;
    next_command.velocity_target_command.fill(0.0f);
    next_command.velocity_profile_direction = profile_direction;
    next_command.velocity_profile_peak = profile_peak_velocity;
    next_command.velocity_profile_accel_time = profile_accel_time;
    next_command.velocity_profile_cruise_time = profile_cruise_time;
    next_command.velocity_profile_total_time = profile_total_time;
    next_command.velocity_profile_duration_sec = static_cast<float>(profile_duration);
    next_command.target_position = target_position;

    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_ = next_command;
    }

    std::cout << "[robot] velocity trapezoid profile target: ";
    for (int i = 0; i < kNumJoints; ++i)
    {
        std::cout << target_position[i] << (i + 1 < kNumJoints ? ", " : "");
    }
    std::cout << " tolerance=" << next_command.velocity_tolerance_deg
              << " profile_duration=" << next_command.velocity_profile_duration_sec
              << " timeout=" << next_command.velocity_timeout_sec
              << "s tt=" << next_command.command_time_sec << "s\n";
    std::cout << "[robot] velocity target start position: ";
    for (int i = 0; i < kNumJoints; ++i)
    {
        std::cout << current_position[i] << (i + 1 < kNumJoints ? ", " : "");
    }
    std::cout << " initial_max_error=" << initial_max_error << "deg\n";
    return true;
}

bool DoosanController::loadTaskTrajectoryCsv(
    const std::string &csv_path,
    float max_joint_step_deg,
    float max_joint_velocity_deg_s,
    float max_joint_acceleration_deg_s2,
    std::vector<JointTrajectoryPoint> *trajectory)
{
    if (!trajectory)
    {
        return false;
    }
    trajectory->clear();

    std::ifstream file(csv_path);
    if (!file)
    {
        std::cerr << "[robot] failed to open trajectory CSV: " << csv_path << '\n';
        return false;
    }

    std::string line;
    if (!std::getline(file, line))
    {
        std::cerr << "[robot] empty trajectory CSV: " << csv_path << '\n';
        return false;
    }

    const auto header = splitCsvLine(line);
    std::unordered_map<std::string, std::size_t> columns;
    for (std::size_t i = 0; i < header.size(); ++i)
    {
        columns[header[i]] = i;
    }

    const char *required[] = {
        "t",
        "position_x",
        "position_y",
        "position_z",
        "orientation_x",
        "orientation_y",
        "orientation_z",
        "orientation_w"};
    for (const auto *name : required)
    {
        if (columns.find(name) == columns.end())
        {
            std::cerr << "[robot] trajectory CSV missing column: " << name << '\n';
            return false;
        }
    }

    JointArray current_joint = {};
    if (!readCurrentJointState(&current_joint, nullptr))
    {
        std::cerr << "[robot] failed to read current joint state before IK\n";
        return false;
    }

    std::vector<double> times;
    std::vector<JointArray> positions;
    JointArray previous_joint = current_joint;
    int previous_solution = -1;
    int previous_row_index = std::numeric_limits<int>::min();
    int parsed_rows = 0;

    while (std::getline(file, line))
    {
        if (trim(line).empty())
        {
            continue;
        }

        const auto fields = splitCsvLine(line);
        double t = 0.0;
        double px = 0.0;
        double py = 0.0;
        double pz = 0.0;
        double qx = 0.0;
        double qy = 0.0;
        double qz = 0.0;
        double qw = 1.0;
        if (!parseDouble(fields, columns, "t", &t) ||
            !parseDouble(fields, columns, "position_x", &px) ||
            !parseDouble(fields, columns, "position_y", &py) ||
            !parseDouble(fields, columns, "position_z", &pz) ||
            !parseDouble(fields, columns, "orientation_x", &qx) ||
            !parseDouble(fields, columns, "orientation_y", &qy) ||
            !parseDouble(fields, columns, "orientation_z", &qz) ||
            !parseDouble(fields, columns, "orientation_w", &qw))
        {
            std::cerr << "[robot] invalid numeric trajectory CSV row: " << line << '\n';
            return false;
        }

        if (columns.find("row_index") != columns.end())
        {
            double row_value = 0.0;
            if (parseDouble(fields, columns, "row_index", &row_value))
            {
                const int row_index = static_cast<int>(row_value);
                if (previous_row_index == std::numeric_limits<int>::min())
                {
                    previous_row_index = row_index;
                }
                else if (row_index != previous_row_index)
                {
                    std::cerr << "[robot] CSV contains multiple row_index values. "
                              << "Pass one trajectory_XXX.csv file at a time.\n";
                    return false;
                }
            }
        }

        if (!times.empty() && t <= times.back())
        {
            std::cerr << "[robot] trajectory time must be strictly increasing. "
                      << "t=" << t << " previous=" << times.back() << '\n';
            return false;
        }

        auto task_pose = taskPoseFromCsvPoseMetersAndQuatXyzw(px, py, pz, qx, qy, qz, qw);

        JointArray best_joint = {};
        double best_score = std::numeric_limits<double>::infinity();
        int best_status = std::numeric_limits<int>::max();
        int best_solution = -1;
        bool used_fallback_search = positions.empty();

        {
            std::lock_guard<std::mutex> lock(drfl_mutex_);
            auto try_solution = [&](int solution, JointArray *candidate) {
                if (!candidate)
                {
                    return false;
                }

                auto *response = drfl_.ikin(
                    task_pose.data(),
                    static_cast<unsigned char>(solution),
                    COORDINATE_SYSTEM_BASE,
                    static_cast<unsigned char>(1));
                if (!response)
                {
                    return false;
                }

                best_status = std::min(best_status, response->_iStatus);
                if (response->_iStatus != 0)
                {
                    return false;
                }

                for (int i = 0; i < kNumJoints; ++i)
                {
                    (*candidate)[i] = response->_fTargetPos[i];
                }
                return true;
            };

            if (!positions.empty() && previous_solution >= 0)
            {
                JointArray candidate = {};
                if (try_solution(previous_solution, &candidate))
                {
                    const double max_step = maxAbsJointDelta(candidate, previous_joint);
                    if (max_step <= static_cast<double>(max_joint_step_deg))
                    {
                        best_joint = candidate;
                        best_score = l2JointDelta(candidate, previous_joint);
                        best_solution = previous_solution;
                    }
                    else
                    {
                        used_fallback_search = true;
                    }
                }
                else
                {
                    used_fallback_search = true;
                }
            }

            if (best_solution < 0)
            {
                used_fallback_search = true;
                for (int solution = 0; solution < 8; ++solution)
                {
                    JointArray candidate = {};
                    if (!try_solution(solution, &candidate))
                    {
                        continue;
                    }

                    const double score = l2JointDelta(candidate, previous_joint);
                    if (score < best_score)
                    {
                        best_score = score;
                        best_joint = candidate;
                        best_solution = solution;
                    }
                }
            }

            if (best_solution >= 0 && parsed_rows < 10)
            {
                auto *fk = drfl_.fkin(best_joint.data(), COORDINATE_SYSTEM_BASE);
                if (!fk)
                {
                    std::cerr << "[robot] IK/FK check row=" << parsed_rows
                              << " solution=" << best_solution
                              << " fkin failed\n";
                }
                else
                {
                    double max_position_error_mm = 0.0;
                    double max_orientation_error_deg = 0.0;
                    for (int i = 0; i < 3; ++i)
                    {
                        max_position_error_mm = std::max(
                            max_position_error_mm,
                            std::fabs(
                                static_cast<double>(task_pose[i]) -
                                static_cast<double>(fk->_fPosition[i])));
                    }
                    for (int i = 3; i < 6; ++i)
                    {
                        max_orientation_error_deg = std::max(
                            max_orientation_error_deg,
                            absAngleDiffDeg(
                                static_cast<double>(task_pose[i]),
                                static_cast<double>(fk->_fPosition[i])));
                    }

                    std::cout << "[robot] IK/FK check row=" << parsed_rows
                              << " solution=" << best_solution
                              << " pos_err_mm=" << max_position_error_mm
                              << " ori_err_deg=" << max_orientation_error_deg
                              << " task_pose=";
                    for (int i = 0; i < kNumJoints; ++i)
                    {
                        std::cout << task_pose[i] << (i + 1 < kNumJoints ? ", " : "");
                    }
                    std::cout << " fk_pose=";
                    for (int i = 0; i < kNumJoints; ++i)
                    {
                        std::cout << fk->_fPosition[i] << (i + 1 < kNumJoints ? ", " : "\n");
                    }
                }
            }
        }

        if (best_solution < 0)
        {
            std::cerr << "[robot] IK failed at CSV row " << parsed_rows
                      << " status=" << best_status << " task_pose(mm,deg)=";
            for (int i = 0; i < kNumJoints; ++i)
            {
                std::cerr << task_pose[i] << (i + 1 < kNumJoints ? ", " : "\n");
            }
            return false;
        }

        const double max_step = maxAbsJointDelta(best_joint, previous_joint);
        if (!positions.empty() && max_step > static_cast<double>(max_joint_step_deg))
        {
            std::cerr << "[robot] IK continuity check failed at CSV row " << parsed_rows
                      << " max_joint_step=" << max_step
                      << "deg limit=" << max_joint_step_deg << "deg\n";
            return false;
        }

        times.push_back(t);
        positions.push_back(best_joint);
        previous_joint = best_joint;
        previous_solution = best_solution;
        ++parsed_rows;

        if (parsed_rows % 100 == 0)
        {
            std::cout << "[robot] IK progress rows=" << parsed_rows
                      << " latest_t=" << t
                      << "s solution=" << best_solution
                      << " fallback=" << (used_fallback_search ? "yes" : "no")
                      << '\n';
        }
    }

    if (positions.size() < 2)
    {
        std::cerr << "[robot] trajectory CSV needs at least two points\n";
        return false;
    }

    std::vector<JointArray> velocities(positions.size());
    std::vector<JointArray> accelerations(positions.size());
    for (std::size_t idx = 0; idx < positions.size(); ++idx)
    {
        const std::size_t lo = idx == 0 ? 0 : idx - 1;
        const std::size_t hi = idx + 1 < positions.size() ? idx + 1 : idx;
        const double dt = times[hi] - times[lo];
        if (dt <= 0.0)
        {
            std::cerr << "[robot] invalid dt while calculating qdot\n";
            return false;
        }

        for (int joint = 0; joint < kNumJoints; ++joint)
        {
            velocities[idx][joint] = static_cast<float>(
                (static_cast<double>(positions[hi][joint]) -
                 static_cast<double>(positions[lo][joint])) /
                dt);
        }
    }

    for (std::size_t idx = 0; idx < positions.size(); ++idx)
    {
        const std::size_t lo = idx == 0 ? 0 : idx - 1;
        const std::size_t hi = idx + 1 < positions.size() ? idx + 1 : idx;
        const double dt = times[hi] - times[lo];
        if (dt <= 0.0)
        {
            std::cerr << "[robot] invalid dt while calculating qddot\n";
            return false;
        }

        for (int joint = 0; joint < kNumJoints; ++joint)
        {
            accelerations[idx][joint] = static_cast<float>(
                (static_cast<double>(velocities[hi][joint]) -
                 static_cast<double>(velocities[lo][joint])) /
                dt);
        }
    }

    double max_velocity = 0.0;
    double max_acceleration = 0.0;
    for (std::size_t idx = 0; idx < positions.size(); ++idx)
    {
        for (int joint = 0; joint < kNumJoints; ++joint)
        {
            max_velocity = std::max(max_velocity, std::fabs(static_cast<double>(velocities[idx][joint])));
            max_acceleration = std::max(max_acceleration, std::fabs(static_cast<double>(accelerations[idx][joint])));
        }
    }

    if (max_velocity > static_cast<double>(max_joint_velocity_deg_s))
    {
        std::cerr << "[robot] qdot check failed max=" << max_velocity
                  << "deg/s limit=" << max_joint_velocity_deg_s << "deg/s\n";
        return false;
    }
    if (max_acceleration > static_cast<double>(max_joint_acceleration_deg_s2))
    {
        std::cerr << "[robot] qddot check failed max=" << max_acceleration
                  << "deg/s^2 limit=" << max_joint_acceleration_deg_s2 << "deg/s^2\n";
        return false;
    }

    trajectory->reserve(positions.size());
    for (std::size_t idx = 0; idx < positions.size(); ++idx)
    {
        JointTrajectoryPoint point;
        point.time_sec = times[idx] - times.front();
        point.position = positions[idx];
        point.velocity = velocities[idx];
        point.acceleration = accelerations[idx];
        trajectory->push_back(point);
    }

    if (!trajectory->empty())
    {
        trajectory->front().time_sec = 0.0;
        trajectory->front().velocity.fill(0.0f);
        trajectory->front().acceleration.fill(0.0f);
        trajectory->back().velocity.fill(0.0f);
        trajectory->back().acceleration.fill(0.0f);
    }

    std::cout << "[robot] loaded task trajectory CSV: " << csv_path
              << " points=" << trajectory->size()
              << " duration=" << trajectory->back().time_sec
              << " max_qdot=" << max_velocity
              << "deg/s max_qddot=" << max_acceleration
              << "deg/s^2\n";
    return true;
}

bool DoosanController::loadJointTrajectoryCsv(
    const std::string &csv_path,
    float max_joint_step_deg,
    float max_joint_velocity_deg_s,
    float max_joint_acceleration_deg_s2,
    std::vector<JointTrajectoryPoint> *trajectory)
{
    if (!trajectory)
    {
        return false;
    }
    trajectory->clear();

    std::ifstream file(csv_path);
    if (!file)
    {
        std::cerr << "[robot] failed to open joint trajectory CSV: " << csv_path << '\n';
        return false;
    }

    std::string line;
    if (!std::getline(file, line))
    {
        std::cerr << "[robot] empty joint trajectory CSV: " << csv_path << '\n';
        return false;
    }

    const auto header = splitCsvLine(line);
    std::unordered_map<std::string, std::size_t> columns;
    for (std::size_t i = 0; i < header.size(); ++i)
    {
        columns[header[i]] = i;
    }

    if (columns.find("t") == columns.end())
    {
        std::cerr << "[robot] joint trajectory CSV missing column: t\n";
        return false;
    }

    std::array<std::string, kNumJoints> position_columns = {};
    std::array<std::string, kNumJoints> velocity_columns = {};
    std::array<std::string, kNumJoints> acceleration_columns = {};
    int velocity_column_count = 0;
    int acceleration_column_count = 0;

    for (int joint = 0; joint < kNumJoints; ++joint)
    {
        position_columns[joint] = findFirstColumn(columns, jointPositionColumnNames(joint));
        if (position_columns[joint].empty())
        {
            std::cerr << "[robot] joint trajectory CSV missing q" << (joint + 1)
                      << " column. Accepted examples: q" << (joint + 1)
                      << ", q" << (joint + 1) << "_deg, joint_" << (joint + 1) << "\n";
            return false;
        }

        velocity_columns[joint] = findFirstColumn(columns, jointVelocityColumnNames(joint));
        if (!velocity_columns[joint].empty())
        {
            ++velocity_column_count;
        }

        acceleration_columns[joint] = findFirstColumn(columns, jointAccelerationColumnNames(joint));
        if (!acceleration_columns[joint].empty())
        {
            ++acceleration_column_count;
        }
    }

    if (velocity_column_count != 0 && velocity_column_count != kNumJoints)
    {
        std::cerr << "[robot] joint trajectory CSV must provide all velocity columns or none\n";
        return false;
    }
    if (acceleration_column_count != 0 && acceleration_column_count != kNumJoints)
    {
        std::cerr << "[robot] joint trajectory CSV must provide all acceleration columns or none\n";
        return false;
    }

    const bool has_velocity = velocity_column_count == kNumJoints;
    const bool has_acceleration = acceleration_column_count == kNumJoints;

    std::vector<double> times;
    std::vector<JointArray> positions;
    std::vector<JointArray> velocities;
    std::vector<JointArray> accelerations;
    int parsed_rows = 0;

    while (std::getline(file, line))
    {
        if (trim(line).empty())
        {
            continue;
        }

        const auto fields = splitCsvLine(line);
        double t = 0.0;
        if (!parseDouble(fields, columns, "t", &t))
        {
            std::cerr << "[robot] invalid t in joint trajectory CSV row: " << line << '\n';
            return false;
        }
        if (!times.empty() && t <= times.back())
        {
            std::cerr << "[robot] joint trajectory time must be strictly increasing. "
                      << "t=" << t << " previous=" << times.back() << '\n';
            return false;
        }

        JointArray position = {};
        JointArray velocity = {};
        JointArray acceleration = {};
        for (int joint = 0; joint < kNumJoints; ++joint)
        {
            double value = 0.0;
            if (!parseDouble(fields, columns, position_columns[joint], &value))
            {
                std::cerr << "[robot] invalid q" << (joint + 1)
                          << " in joint trajectory CSV row: " << line << '\n';
                return false;
            }
            position[joint] = static_cast<float>(value);

            if (has_velocity)
            {
                if (!parseDouble(fields, columns, velocity_columns[joint], &value))
                {
                    std::cerr << "[robot] invalid qd" << (joint + 1)
                              << " in joint trajectory CSV row: " << line << '\n';
                    return false;
                }
                velocity[joint] = static_cast<float>(value);
            }

            if (has_acceleration)
            {
                if (!parseDouble(fields, columns, acceleration_columns[joint], &value))
                {
                    std::cerr << "[robot] invalid qdd" << (joint + 1)
                              << " in joint trajectory CSV row: " << line << '\n';
                    return false;
                }
                acceleration[joint] = static_cast<float>(value);
            }
        }

        if (!positions.empty())
        {
            const double max_step = maxAbsJointDelta(position, positions.back());
            if (max_step > static_cast<double>(max_joint_step_deg))
            {
                std::cerr << "[robot] joint trajectory continuity check failed at row "
                          << parsed_rows << " max_joint_step=" << max_step
                          << "deg limit=" << max_joint_step_deg << "deg\n";
                return false;
            }
        }

        times.push_back(t);
        positions.push_back(position);
        velocities.push_back(velocity);
        accelerations.push_back(acceleration);
        ++parsed_rows;
    }

    if (positions.size() < 2)
    {
        std::cerr << "[robot] joint trajectory CSV needs at least two points\n";
        return false;
    }

    if (!has_velocity)
    {
        velocities.assign(positions.size(), JointArray{});
        for (std::size_t idx = 0; idx < positions.size(); ++idx)
        {
            const std::size_t lo = idx == 0 ? 0 : idx - 1;
            const std::size_t hi = idx + 1 < positions.size() ? idx + 1 : idx;
            const double dt = times[hi] - times[lo];
            if (dt <= 0.0)
            {
                std::cerr << "[robot] invalid dt while calculating joint qdot\n";
                return false;
            }
            for (int joint = 0; joint < kNumJoints; ++joint)
            {
                velocities[idx][joint] = static_cast<float>(
                    (static_cast<double>(positions[hi][joint]) -
                     static_cast<double>(positions[lo][joint])) /
                    dt);
            }
        }
    }

    if (!has_acceleration)
    {
        accelerations.assign(positions.size(), JointArray{});
        for (std::size_t idx = 0; idx < positions.size(); ++idx)
        {
            const std::size_t lo = idx == 0 ? 0 : idx - 1;
            const std::size_t hi = idx + 1 < positions.size() ? idx + 1 : idx;
            const double dt = times[hi] - times[lo];
            if (dt <= 0.0)
            {
                std::cerr << "[robot] invalid dt while calculating joint qddot\n";
                return false;
            }
            for (int joint = 0; joint < kNumJoints; ++joint)
            {
                accelerations[idx][joint] = static_cast<float>(
                    (static_cast<double>(velocities[hi][joint]) -
                     static_cast<double>(velocities[lo][joint])) /
                    dt);
            }
        }
    }

    double max_velocity = 0.0;
    double max_acceleration = 0.0;
    for (std::size_t idx = 0; idx < positions.size(); ++idx)
    {
        for (int joint = 0; joint < kNumJoints; ++joint)
        {
            max_velocity = std::max(max_velocity, std::fabs(static_cast<double>(velocities[idx][joint])));
            max_acceleration = std::max(max_acceleration, std::fabs(static_cast<double>(accelerations[idx][joint])));
        }
    }

    if (max_velocity > static_cast<double>(max_joint_velocity_deg_s))
    {
        std::cerr << "[robot] optimized qdot check failed max=" << max_velocity
                  << "deg/s limit=" << max_joint_velocity_deg_s << "deg/s\n";
        return false;
    }
    if (max_acceleration > static_cast<double>(max_joint_acceleration_deg_s2))
    {
        std::cerr << "[robot] optimized qddot check failed max=" << max_acceleration
                  << "deg/s^2 limit=" << max_joint_acceleration_deg_s2 << "deg/s^2\n";
        return false;
    }

    trajectory->reserve(positions.size());
    for (std::size_t idx = 0; idx < positions.size(); ++idx)
    {
        JointTrajectoryPoint point;
        point.time_sec = times[idx] - times.front();
        point.position = positions[idx];
        point.velocity = velocities[idx];
        point.acceleration = accelerations[idx];
        trajectory->push_back(point);
    }

    trajectory->front().time_sec = 0.0;
    trajectory->front().velocity.fill(0.0f);
    trajectory->front().acceleration.fill(0.0f);
    trajectory->back().velocity.fill(0.0f);
    trajectory->back().acceleration.fill(0.0f);

    std::cout << "[robot] loaded optimized joint trajectory CSV: " << csv_path
              << " points=" << trajectory->size()
              << " duration=" << trajectory->back().time_sec
              << "s max_qdot=" << max_velocity
              << "deg/s max_qddot=" << max_acceleration
              << "deg/s^2\n";
    return true;
}

bool DoosanController::loadCartesianTaskTrajectoryCsv(
    const std::string &csv_path,
    std::vector<TaskTrajectoryPoint> *trajectory)
{
    if (!trajectory)
    {
        return false;
    }
    trajectory->clear();

    std::ifstream file(csv_path);
    if (!file)
    {
        std::cerr << "[robot] failed to open task trajectory CSV: " << csv_path << '\n';
        return false;
    }

    std::string line;
    if (!std::getline(file, line))
    {
        std::cerr << "[robot] empty task trajectory CSV: " << csv_path << '\n';
        return false;
    }

    const auto header = splitCsvLine(line);
    std::unordered_map<std::string, std::size_t> columns;
    for (std::size_t i = 0; i < header.size(); ++i)
    {
        columns[header[i]] = i;
    }

    const char *required[] = {
        "t",
        "position_x",
        "position_y",
        "position_z",
        "orientation_x",
        "orientation_y",
        "orientation_z",
        "orientation_w",
        "linear_velocity_x",
        "linear_velocity_y",
        "linear_velocity_z",
        "angular_velocity_x",
        "angular_velocity_y",
        "angular_velocity_z"};
    for (const auto *name : required)
    {
        if (columns.find(name) == columns.end())
        {
            std::cerr << "[robot] task trajectory CSV missing column: " << name << '\n';
            return false;
        }
    }

    int previous_row_index = std::numeric_limits<int>::min();
    double first_t = 0.0;
    double previous_t = -std::numeric_limits<double>::infinity();
    int parsed_rows = 0;

    while (std::getline(file, line))
    {
        if (trim(line).empty())
        {
            continue;
        }

        const auto fields = splitCsvLine(line);
        double t = 0.0;
        double px = 0.0;
        double py = 0.0;
        double pz = 0.0;
        double qx = 0.0;
        double qy = 0.0;
        double qz = 0.0;
        double qw = 1.0;
        double vx = 0.0;
        double vy = 0.0;
        double vz = 0.0;
        double wx = 0.0;
        double wy = 0.0;
        double wz = 0.0;
        if (!parseDouble(fields, columns, "t", &t) ||
            !parseDouble(fields, columns, "position_x", &px) ||
            !parseDouble(fields, columns, "position_y", &py) ||
            !parseDouble(fields, columns, "position_z", &pz) ||
            !parseDouble(fields, columns, "orientation_x", &qx) ||
            !parseDouble(fields, columns, "orientation_y", &qy) ||
            !parseDouble(fields, columns, "orientation_z", &qz) ||
            !parseDouble(fields, columns, "orientation_w", &qw) ||
            !parseDouble(fields, columns, "linear_velocity_x", &vx) ||
            !parseDouble(fields, columns, "linear_velocity_y", &vy) ||
            !parseDouble(fields, columns, "linear_velocity_z", &vz) ||
            !parseDouble(fields, columns, "angular_velocity_x", &wx) ||
            !parseDouble(fields, columns, "angular_velocity_y", &wy) ||
            !parseDouble(fields, columns, "angular_velocity_z", &wz))
        {
            std::cerr << "[robot] invalid numeric task trajectory CSV row: " << line << '\n';
            return false;
        }

        if (columns.find("row_index") != columns.end())
        {
            double row_value = 0.0;
            if (parseDouble(fields, columns, "row_index", &row_value))
            {
                const int row_index = static_cast<int>(row_value);
                if (previous_row_index == std::numeric_limits<int>::min())
                {
                    previous_row_index = row_index;
                }
                else if (row_index != previous_row_index)
                {
                    std::cerr << "[robot] CSV contains multiple row_index values. "
                              << "Pass one trajectory_XXX.csv file at a time.\n";
                    return false;
                }
            }
        }

        if (parsed_rows == 0)
        {
            first_t = t;
        }
        else if (t <= previous_t)
        {
            std::cerr << "[robot] task trajectory time must be strictly increasing. "
                      << "t=" << t << " previous=" << previous_t << '\n';
            return false;
        }

        const std::array<double, 3> current_position = {px, py, pz};
        TaskTrajectoryPoint point;
        point.time_sec = t - first_t;
        point.position = current_position;
        point.rotation = quaternionXyzwToRotation(qx, qy, qz, qw);
        point.linear_velocity = {vx, vy, vz};
        point.angular_velocity = {wx, wy, wz};
        point.doosan_task_pose = taskPoseFromCsvPoseMetersAndQuatXyzw(px, py, pz, qx, qy, qz, qw);

        trajectory->push_back(point);
        previous_t = t;
        ++parsed_rows;
    }

    if (trajectory->size() < 2)
    {
        std::cerr << "[robot] task trajectory CSV needs at least two points\n";
        return false;
    }

    trajectory->front().time_sec = 0.0;
    std::cout << "[robot] loaded cartesian task trajectory CSV: " << csv_path
              << " points=" << trajectory->size()
              << " duration=" << trajectory->back().time_sec << "s\n";
    return true;
}

bool DoosanController::solveClosestIk(
    const std::array<float, kNumJoints> &task_pose,
    const JointArray &reference_joint,
    JointArray *joint,
    int *solution_space)
{
    if (!joint)
    {
        return false;
    }

    JointArray best_joint = {};
    double best_score = std::numeric_limits<double>::infinity();
    int best_solution = -1;
    int best_status = std::numeric_limits<int>::max();

    std::lock_guard<std::mutex> lock(drfl_mutex_);
    for (int solution = 0; solution < 8; ++solution)
    {
        auto *response = drfl_.ikin(
            const_cast<float *>(task_pose.data()),
            static_cast<unsigned char>(solution),
            COORDINATE_SYSTEM_BASE,
            static_cast<unsigned char>(1));
        if (!response)
        {
            continue;
        }

        best_status = std::min(best_status, response->_iStatus);
        if (response->_iStatus != 0)
        {
            continue;
        }

        JointArray candidate = {};
        for (int i = 0; i < kNumJoints; ++i)
        {
            candidate[i] = response->_fTargetPos[i];
        }

        const double score = l2JointDelta(candidate, reference_joint);
        if (score < best_score)
        {
            best_score = score;
            best_joint = candidate;
            best_solution = solution;
        }
    }

    if (best_solution < 0)
    {
        std::cerr << "[robot] closest IK failed status=" << best_status << " task_pose(mm,deg)=";
        for (int i = 0; i < kNumJoints; ++i)
        {
            std::cerr << task_pose[i] << (i + 1 < kNumJoints ? ", " : "\n");
        }
        return false;
    }

    *joint = best_joint;
    if (solution_space)
    {
        *solution_space = best_solution;
    }
    return true;
}

bool DoosanController::setTaskTrajectoryCsvPositionTarget(
    const std::string &csv_path,
    float command_time_sec,
    float max_joint_step_deg,
    float max_joint_velocity_deg_s,
    float max_joint_acceleration_deg_s2)
{
    auto trajectory = std::make_shared<std::vector<JointTrajectoryPoint>>();
    if (!loadTaskTrajectoryCsv(
            csv_path,
            max_joint_step_deg > 0.0f ? max_joint_step_deg : 20.0f,
            max_joint_velocity_deg_s > 0.0f ? max_joint_velocity_deg_s : 250.0f,
            max_joint_acceleration_deg_s2 > 0.0f ? max_joint_acceleration_deg_s2 : 1500.0f,
            trajectory.get()))
    {
        return false;
    }

    return setJointTrajectoryPositionTarget(trajectory, command_time_sec);
}

bool DoosanController::setJointTrajectoryPositionTarget(
    std::shared_ptr<const std::vector<JointTrajectoryPoint>> trajectory,
    float command_time_sec)
{
    if (!trajectory || trajectory->empty())
    {
        std::cerr << "[robot] empty joint trajectory target\n";
        return false;
    }

    Command next_command;
    next_command.mode = CommandMode::JointTrajectory;
    next_command.command_time_sec = commandTimeOrDefault(command_time_sec, config_.command_time_sec);
    next_command.trajectory_active = true;
    next_command.stop_rt_when_done = true;
    next_command.trajectory_start = Clock::now();
    next_command.trajectory_duration_sec = static_cast<float>(trajectory->back().time_sec);
    next_command.position = trajectory->front().position;
    next_command.velocity = trajectory->front().velocity;
    next_command.acceleration = trajectory->front().acceleration;
    next_command.target_position = trajectory->back().position;
    next_command.joint_trajectory = trajectory;

    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_ = next_command;
    }

    std::cout << "[robot] task trajectory target ready duration="
              << next_command.trajectory_duration_sec
              << "s tt=" << next_command.command_time_sec << "s\n";
    return true;
}

bool DoosanController::setJointTrajectoryVelocityTarget(
    std::shared_ptr<const std::vector<JointTrajectoryPoint>> trajectory,
    float command_time_sec,
    float position_gain,
    float max_joint_velocity_deg_s,
    float max_joint_acceleration_deg_s2)
{
    if (!trajectory || trajectory->empty())
    {
        std::cerr << "[robot] empty joint velocity trajectory target\n";
        return false;
    }

    Command next_command;
    next_command.mode = CommandMode::JointVelocityTrajectory;
    next_command.command_time_sec = commandTimeOrDefault(command_time_sec, config_.command_time_sec);
    next_command.trajectory_active = true;
    next_command.stop_rt_when_done = true;
    next_command.trajectory_start = Clock::now();
    next_command.trajectory_duration_sec = static_cast<float>(trajectory->back().time_sec);
    next_command.position_gain = position_gain > 0.0f ? position_gain : 4.0f;
    next_command.max_joint_velocity_deg_s =
        max_joint_velocity_deg_s > 0.0f ? max_joint_velocity_deg_s : 70.0f;
    next_command.max_joint_acceleration_deg_s2 =
        max_joint_acceleration_deg_s2 > 0.0f ? max_joint_acceleration_deg_s2 : 500.0f;
    next_command.position = trajectory->front().position;
    next_command.velocity = trajectory->front().velocity;
    next_command.acceleration = trajectory->front().acceleration;
    next_command.target_position = trajectory->back().position;
    next_command.joint_trajectory = trajectory;
    next_command.previous_velocity_command.fill(0.0f);
    next_command.velocity_target_last_update = Clock::now();

    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_ = next_command;
    }

    std::cout << "[robot] joint velocity trajectory target ready duration="
              << next_command.trajectory_duration_sec
              << "s tt=" << next_command.command_time_sec
              << "s Kp=" << next_command.position_gain
              << " max_qdot=" << next_command.max_joint_velocity_deg_s
              << "deg/s max_qddot=" << next_command.max_joint_acceleration_deg_s2
              << "deg/s^2\n";
    return true;
}

bool DoosanController::setTaskVelocityTrajectoryTarget(
    std::shared_ptr<const std::vector<TaskTrajectoryPoint>> trajectory,
    std::shared_ptr<CartesianVelocityContext> context,
    float command_time_sec,
    float position_gain,
    float orientation_gain,
    float damping,
    float max_joint_velocity_deg_s,
    float max_joint_acceleration_deg_s2)
{
    if (!trajectory || trajectory->empty())
    {
        std::cerr << "[robot] empty task velocity trajectory target\n";
        return false;
    }
    if (!context)
    {
        std::cerr << "[robot] missing cartesian velocity context\n";
        return false;
    }

    Command next_command;
    next_command.mode = CommandMode::TaskVelocityTrajectory;
    next_command.command_time_sec = commandTimeOrDefault(command_time_sec, config_.command_time_sec);
    next_command.trajectory_active = true;
    next_command.stop_rt_when_done = true;
    next_command.trajectory_start = Clock::now();
    next_command.trajectory_duration_sec = static_cast<float>(trajectory->back().time_sec);
    next_command.position_gain = position_gain > 0.0f ? position_gain : 8.0f;
    next_command.orientation_gain = orientation_gain > 0.0f ? orientation_gain : 4.0f;
    next_command.damping = damping > 0.0f ? damping : 0.03f;
    next_command.max_joint_velocity_deg_s =
        max_joint_velocity_deg_s > 0.0f ? max_joint_velocity_deg_s : 70.0f;
    next_command.max_joint_acceleration_deg_s2 =
        max_joint_acceleration_deg_s2 > 0.0f ? max_joint_acceleration_deg_s2 : 500.0f;
    next_command.velocity_target_last_update = Clock::now();
    next_command.task_trajectory = trajectory;
    next_command.cartesian_velocity_context = context;
    next_command.previous_velocity_command.fill(0.0f);

    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_ = next_command;
    }

    std::cout << "[robot] task velocity trajectory target ready duration="
              << next_command.trajectory_duration_sec
              << "s tt=" << next_command.command_time_sec
              << "s max_qdot=" << next_command.max_joint_velocity_deg_s
              << "deg/s max_qddot=" << next_command.max_joint_acceleration_deg_s2
              << "deg/s^2\n";
    return true;
}

void DoosanController::realtimeLoop()
{
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(std::max(config_.rt_period_sec, 0.001f)));

    while (realtime_loop_running_ && !realtime_control_running_)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    auto next = std::chrono::steady_clock::now();

    while (realtime_loop_running_ && realtime_control_running_)
    {
        next += period;

        Command command_snapshot;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            command_snapshot = command_;
        }

        if (command_snapshot.mode == CommandMode::JointPosition)
        {
            JointArray position = command_snapshot.position;
            JointArray velocity = command_snapshot.velocity;
            JointArray acceleration = command_snapshot.acceleration;
            bool trajectory_done = false;

            if (command_snapshot.trajectory_active)
            {
                const double elapsed_sec = std::chrono::duration<double>(
                    Clock::now() - command_snapshot.trajectory_start)
                                               .count();
                const double tf = std::max(
                    static_cast<double>(command_snapshot.trajectory_duration_sec),
                    static_cast<double>(config_.rt_period_sec));
                const double t = std::clamp(elapsed_sec, 0.0, tf);
                const double t2 = t * t;
                const double t3 = t2 * t;
                const double t4 = t3 * t;
                const double t5 = t4 * t;
                trajectory_done = elapsed_sec >= tf;

                for (int i = 0; i < kNumJoints; ++i)
                {
                    const double a0 = command_snapshot.a0[i];
                    const double a1 = command_snapshot.a1[i];
                    const double a2 = command_snapshot.a2[i];
                    const double a3 = command_snapshot.a3[i];
                    const double a4 = command_snapshot.a4[i];
                    const double a5 = command_snapshot.a5[i];

                    position[i] = static_cast<float>(
                        a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5);
                    velocity[i] = static_cast<float>(
                        a1 + 2.0 * a2 * t + 3.0 * a3 * t2 + 4.0 * a4 * t3 +
                        5.0 * a5 * t4);
                    acceleration[i] = static_cast<float>(
                        2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * t2 + 20.0 * a5 * t3);

                    if (trajectory_done)
                    {
                        position[i] = command_snapshot.target_position[i];
                        velocity[i] = command_snapshot.velocity[i];
                        acceleration[i] = command_snapshot.acceleration[i];
                    }
                }
            }

            std::lock_guard<std::mutex> lock(drfl_mutex_);
            drfl_.servoj_rt(
                position.data(),
                velocity.data(),
                acceleration.data(),
                command_snapshot.command_time_sec);

            if (trajectory_done && command_snapshot.stop_rt_when_done)
            {
                std::lock_guard<std::mutex> command_lock(command_mutex_);
                command_.mode = CommandMode::Hold;
                command_.trajectory_active = false;
                command_.stop_rt_when_done = false;
                realtime_loop_running_ = false;
            }
        }
        else if (command_snapshot.mode == CommandMode::JointTrajectory)
        {
            JointArray position = command_snapshot.position;
            JointArray velocity = command_snapshot.velocity;
            JointArray acceleration = command_snapshot.acceleration;
            bool trajectory_done = false;

            const auto trajectory = command_snapshot.joint_trajectory;
            if (trajectory && !trajectory->empty())
            {
                const double elapsed_sec = std::chrono::duration<double>(
                    Clock::now() - command_snapshot.trajectory_start)
                                               .count();
                const double duration = trajectory->back().time_sec;
                trajectory_done = elapsed_sec >= duration;

                if (trajectory_done || trajectory->size() == 1)
                {
                    position = trajectory->back().position;
                    velocity = trajectory->back().velocity;
                    acceleration = trajectory->back().acceleration;
                }
                else
                {
                    const auto upper = std::lower_bound(
                        trajectory->begin(),
                        trajectory->end(),
                        elapsed_sec,
                        [](const JointTrajectoryPoint &point, double t) {
                            return point.time_sec < t;
                        });

                    if (upper == trajectory->begin())
                    {
                        position = upper->position;
                        velocity = upper->velocity;
                        acceleration = upper->acceleration;
                    }
                    else if (upper == trajectory->end())
                    {
                        position = trajectory->back().position;
                        velocity = trajectory->back().velocity;
                        acceleration = trajectory->back().acceleration;
                    }
                    else
                    {
                        const auto lower = upper - 1;
                        const double segment_dt = std::max(
                            upper->time_sec - lower->time_sec,
                            1.0e-9);
                        const double alpha = (elapsed_sec - lower->time_sec) / segment_dt;
                        position = interpolateJointArray(lower->position, upper->position, alpha);
                        velocity = interpolateJointArray(lower->velocity, upper->velocity, alpha);
                        acceleration = interpolateJointArray(lower->acceleration, upper->acceleration, alpha);
                    }
                }
            }
            else
            {
                trajectory_done = true;
            }

            {
                std::lock_guard<std::mutex> lock(drfl_mutex_);
                drfl_.servoj_rt(
                    position.data(),
                    velocity.data(),
                    acceleration.data(),
                    command_snapshot.command_time_sec);
            }

            if (trajectory_done && command_snapshot.stop_rt_when_done)
            {
                std::cout << "[robot] task trajectory done\n";
                std::lock_guard<std::mutex> command_lock(command_mutex_);
                command_.mode = CommandMode::Hold;
                command_.trajectory_active = false;
                command_.stop_rt_when_done = false;
                command_.position = position;
                command_.velocity = velocity;
                command_.acceleration = acceleration;
                command_.joint_trajectory.reset();
                realtime_loop_running_ = false;
            }
        }
        else if (command_snapshot.mode == CommandMode::JointVelocityTrajectory)
        {
            JointArray current_position = command_snapshot.position;
            JointArray position_ref = command_snapshot.position;
            JointArray velocity_ref = command_snapshot.velocity;
            JointArray velocity = {};
            JointArray acceleration = {};
            bool trajectory_done = false;
            const auto now = Clock::now();
            double dt = static_cast<double>(config_.rt_period_sec);
            if (command_snapshot.velocity_target_last_update != Clock::time_point{})
            {
                dt = std::max(
                    std::chrono::duration<double>(
                        now - command_snapshot.velocity_target_last_update)
                        .count(),
                    static_cast<double>(config_.rt_period_sec));
            }

            const auto trajectory = command_snapshot.joint_trajectory;
            const double elapsed_sec = std::chrono::duration<double>(
                now - command_snapshot.trajectory_start)
                                           .count();

            if (!trajectory || trajectory->empty())
            {
                trajectory_done = true;
            }
            else
            {
                const double duration = trajectory->back().time_sec;
                trajectory_done = elapsed_sec >= duration;

                if (trajectory_done || trajectory->size() == 1)
                {
                    position_ref = trajectory->back().position;
                    velocity_ref.fill(0.0f);
                }
                else
                {
                    const auto upper = std::lower_bound(
                        trajectory->begin(),
                        trajectory->end(),
                        elapsed_sec,
                        [](const JointTrajectoryPoint &point, double t) {
                            return point.time_sec < t;
                        });

                    if (upper == trajectory->begin())
                    {
                        position_ref = upper->position;
                        velocity_ref = upper->velocity;
                    }
                    else if (upper == trajectory->end())
                    {
                        position_ref = trajectory->back().position;
                        velocity_ref = trajectory->back().velocity;
                    }
                    else
                    {
                        const auto lower = upper - 1;
                        const double segment_dt = std::max(
                            upper->time_sec - lower->time_sec,
                            1.0e-9);
                        const double alpha = (elapsed_sec - lower->time_sec) / segment_dt;
                        position_ref = interpolateJointArray(lower->position, upper->position, alpha);
                        velocity_ref = interpolateJointArray(lower->velocity, upper->velocity, alpha);
                    }
                }
            }

            if (!trajectory_done)
            {
                if (!readCurrentJointState(&current_position, nullptr))
                {
                    std::cerr << "[robot] failed to read current joint state during joint velocity trajectory\n";
                    current_position = command_snapshot.position;
                }

                for (int i = 0; i < kNumJoints; ++i)
                {
                    const double position_error =
                        static_cast<double>(position_ref[i]) -
                        static_cast<double>(current_position[i]);
                    velocity[i] = static_cast<float>(
                        static_cast<double>(velocity_ref[i]) +
                        static_cast<double>(command_snapshot.position_gain) * position_error);
                }

                velocity = clampJointArray(
                    velocity,
                    static_cast<double>(command_snapshot.max_joint_velocity_deg_s));

                for (int i = 0; i < kNumJoints; ++i)
                {
                    const double raw_acceleration =
                        (static_cast<double>(velocity[i]) -
                         static_cast<double>(command_snapshot.previous_velocity_command[i])) /
                        std::max(dt, 1.0e-6);
                    const double limited_acceleration = std::clamp(
                        raw_acceleration,
                        -static_cast<double>(command_snapshot.max_joint_acceleration_deg_s2),
                        static_cast<double>(command_snapshot.max_joint_acceleration_deg_s2));
                    acceleration[i] = static_cast<float>(limited_acceleration);
                    if (std::fabs(raw_acceleration - limited_acceleration) > 1.0e-9)
                    {
                        velocity[i] = static_cast<float>(
                            static_cast<double>(command_snapshot.previous_velocity_command[i]) +
                            limited_acceleration * dt);
                    }
                }
            }

            if (trajectory_done)
            {
                velocity.fill(0.0f);
                acceleration.fill(0.0f);
            }

            {
                std::lock_guard<std::mutex> lock(drfl_mutex_);
                drfl_.speedj_rt(
                    velocity.data(),
                    acceleration.data(),
                    command_snapshot.command_time_sec);
            }

            if (trajectory_done && command_snapshot.stop_rt_when_done)
            {
                std::cout << "[robot] joint velocity trajectory done\n";
                std::lock_guard<std::mutex> command_lock(command_mutex_);
                command_.mode = CommandMode::Hold;
                command_.trajectory_active = false;
                command_.stop_rt_when_done = false;
                command_.position = current_position;
                command_.velocity = velocity;
                command_.acceleration = acceleration;
                command_.previous_velocity_command = velocity;
                command_.joint_trajectory.reset();
                realtime_loop_running_ = false;
            }
            else
            {
                std::lock_guard<std::mutex> command_lock(command_mutex_);
                if (command_.mode == CommandMode::JointVelocityTrajectory)
                {
                    command_.position = current_position;
                    command_.velocity = velocity;
                    command_.acceleration = acceleration;
                    command_.previous_velocity_command = velocity;
                    command_.velocity_target_last_update = now;
                }
            }
        }
        else if (command_snapshot.mode == CommandMode::TaskVelocityTrajectory)
        {
            JointArray current_position = command_snapshot.position;
            JointArray velocity = {};
            JointArray acceleration = {};
            bool trajectory_done = false;
            const auto now = Clock::now();
            double dt = static_cast<double>(config_.rt_period_sec);
            if (command_snapshot.velocity_target_last_update != Clock::time_point{})
            {
                dt = std::max(
                    std::chrono::duration<double>(
                        now - command_snapshot.velocity_target_last_update)
                        .count(),
                    static_cast<double>(config_.rt_period_sec));
            }

            const auto trajectory = command_snapshot.task_trajectory;
            const auto context = command_snapshot.cartesian_velocity_context;
            const double elapsed_sec = std::chrono::duration<double>(
                now - command_snapshot.trajectory_start)
                                           .count();

            if (!trajectory || trajectory->empty() || !context)
            {
                trajectory_done = true;
            }
            else
            {
                trajectory_done = elapsed_sec >= trajectory->back().time_sec;
            }

#if DOOSAN_HAS_PINOCCHIO
            if (!trajectory_done)
            {
                if (!readCurrentJointState(&current_position, nullptr))
                {
                    std::cerr << "[robot] failed to read current joint state during task velocity trajectory\n";
                    current_position = command_snapshot.position;
                }

                Eigen::VectorXd q_rad(context->model.nq);
                for (int i = 0; i < kNumJoints; ++i)
                {
                    q_rad[i] = degToRad(static_cast<double>(current_position[i]));
                }

                pinocchio::forwardKinematics(context->model, context->data, q_rad);
                pinocchio::updateFramePlacements(context->model, context->data);

                const auto ref = interpolateTaskTrajectoryPoint(*trajectory, elapsed_sec);
                const auto &current_pose = context->data.oMf[context->frame_id];
                const Eigen::Vector3d p_cur = current_pose.translation();
                const Eigen::Matrix3d R_cur = current_pose.rotation();
                const Eigen::Vector3d p_ref = vectorFromArray3(ref.position);
                const Eigen::Matrix3d R_ref = rotationFromRowMajor(ref.rotation);
                const Eigen::Vector3d v_ref = vectorFromArray3(ref.linear_velocity);
                const Eigen::Vector3d w_ref = vectorFromArray3(ref.angular_velocity);

                const Eigen::Vector3d e_pos = p_ref - p_cur;
                const Eigen::Vector3d e_rot = pinocchio::log3(R_ref * R_cur.transpose());

                Eigen::Matrix<double, 6, 1> v_cmd;
                v_cmd.head<3>() = v_ref + static_cast<double>(command_snapshot.position_gain) * e_pos;
                v_cmd.tail<3>() = w_ref + static_cast<double>(command_snapshot.orientation_gain) * e_rot;

                Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, context->model.nv);
                jacobian.setZero();
                pinocchio::computeFrameJacobian(
                    context->model,
                    context->data,
                    q_rad,
                    context->frame_id,
                    pinocchio::LOCAL_WORLD_ALIGNED,
                    jacobian);

                const double lambda = std::max(static_cast<double>(command_snapshot.damping), 1.0e-6);
                const Eigen::Matrix<double, 6, 6> system =
                    jacobian * jacobian.transpose() +
                    lambda * lambda * Eigen::Matrix<double, 6, 6>::Identity();
                const Eigen::VectorXd qdot_rad =
                    jacobian.transpose() * system.ldlt().solve(v_cmd);

                JointArray unclamped_velocity = {};
                for (int i = 0; i < kNumJoints; ++i)
                {
                    unclamped_velocity[i] = static_cast<float>(radToDeg(qdot_rad[i]));
                }

                velocity = clampJointArray(
                    unclamped_velocity,
                    static_cast<double>(command_snapshot.max_joint_velocity_deg_s));

                for (int i = 0; i < kNumJoints; ++i)
                {
                    const double raw_acceleration =
                        (static_cast<double>(velocity[i]) -
                         static_cast<double>(command_snapshot.previous_velocity_command[i])) /
                        std::max(dt, 1.0e-6);
                    const double limited_acceleration = std::clamp(
                        raw_acceleration,
                        -static_cast<double>(command_snapshot.max_joint_acceleration_deg_s2),
                        static_cast<double>(command_snapshot.max_joint_acceleration_deg_s2));
                    acceleration[i] = static_cast<float>(limited_acceleration);
                    if (std::fabs(raw_acceleration - limited_acceleration) > 1.0e-9)
                    {
                        velocity[i] = static_cast<float>(
                            static_cast<double>(command_snapshot.previous_velocity_command[i]) +
                            limited_acceleration * dt);
                    }
                }
            }
#else
            trajectory_done = true;
#endif

            if (trajectory_done)
            {
                velocity.fill(0.0f);
                acceleration.fill(0.0f);
            }

            {
                std::lock_guard<std::mutex> lock(drfl_mutex_);
                drfl_.speedj_rt(
                    velocity.data(),
                    acceleration.data(),
                    command_snapshot.command_time_sec);
            }

            if (trajectory_done && command_snapshot.stop_rt_when_done)
            {
                std::cout << "[robot] task velocity trajectory done\n";
                std::lock_guard<std::mutex> command_lock(command_mutex_);
                command_.mode = CommandMode::Hold;
                command_.trajectory_active = false;
                command_.stop_rt_when_done = false;
                command_.position = current_position;
                command_.velocity = velocity;
                command_.acceleration = acceleration;
                command_.previous_velocity_command = velocity;
                command_.task_trajectory.reset();
                command_.cartesian_velocity_context.reset();
                realtime_loop_running_ = false;
            }
            else
            {
                std::lock_guard<std::mutex> command_lock(command_mutex_);
                if (command_.mode == CommandMode::TaskVelocityTrajectory)
                {
                    command_.position = current_position;
                    command_.velocity = velocity;
                    command_.acceleration = acceleration;
                    command_.previous_velocity_command = velocity;
                    command_.velocity_target_last_update = now;
                }
            }
        }
        else if (command_snapshot.mode == CommandMode::JointVelocity)
        {
            std::lock_guard<std::mutex> lock(drfl_mutex_);
            drfl_.speedj_rt(
                command_snapshot.velocity.data(),
                command_snapshot.acceleration.data(),
                command_snapshot.command_time_sec);
        }
        else if (command_snapshot.mode == CommandMode::JointVelocityTarget)
        {
            JointArray current_position = command_snapshot.position;
            JointArray velocity = {};
            JointArray acceleration = command_snapshot.acceleration;
            bool target_done = false;
            const auto now = Clock::now();
            const double dt = std::max(
                std::chrono::duration<double>(now - command_snapshot.velocity_target_last_update).count(),
                static_cast<double>(config_.rt_period_sec));
            const double elapsed_sec = std::chrono::duration<double>(
                now - command_snapshot.velocity_target_start)
                                           .count();
            double max_error = 0.0;

            if (!readCurrentJointState(&current_position, nullptr))
            {
                std::cerr << "[robot] failed to read current joint state during velocity target control\n";
                current_position = command_snapshot.position;
            }
            else
            {
                double max_feedback_step = 0.0;
                double max_allowed_step = 5.0;
                for (int i = 0; i < kNumJoints; ++i)
                {
                    max_feedback_step = std::max(
                        max_feedback_step,
                        std::fabs(
                            static_cast<double>(current_position[i]) -
                            static_cast<double>(command_snapshot.position[i])));
                    max_allowed_step = std::max(
                        max_allowed_step,
                        std::fabs(static_cast<double>(command_snapshot.velocity_profile_peak[i])) * dt + 5.0);
                }
                if (max_feedback_step > max_allowed_step)
                {
                    current_position = command_snapshot.position;
                }
            }

            for (int i = 0; i < kNumJoints; ++i)
            {
                const double error =
                    static_cast<double>(command_snapshot.target_position[i]) -
                    static_cast<double>(current_position[i]);
                max_error = std::max(max_error, std::fabs(error));

                const double total_time = command_snapshot.velocity_profile_total_time[i];
                const double accel_time = command_snapshot.velocity_profile_accel_time[i];
                const double cruise_time = command_snapshot.velocity_profile_cruise_time[i];
                const double peak_velocity = command_snapshot.velocity_profile_peak[i];
                const double direction = command_snapshot.velocity_profile_direction[i];

                double speed = 0.0;
                if (total_time > 0.0 && accel_time > 0.0 && elapsed_sec < total_time)
                {
                    if (elapsed_sec < accel_time)
                    {
                        speed = peak_velocity * (elapsed_sec / accel_time);
                    }
                    else if (elapsed_sec < accel_time + cruise_time)
                    {
                        speed = peak_velocity;
                    }
                    else
                    {
                        const double decel_elapsed = elapsed_sec - accel_time - cruise_time;
                        speed = peak_velocity * std::max(
                            0.0,
                            1.0 - decel_elapsed / accel_time);
                    }
                }

                velocity[i] = static_cast<float>(direction * speed);
            }

            const bool timed_out = elapsed_sec >= command_snapshot.velocity_timeout_sec;
            target_done =
                elapsed_sec >= static_cast<double>(command_snapshot.velocity_profile_duration_sec) ||
                timed_out;

            if (target_done)
            {
                std::cout << "[robot] velocity trapezoid profile done max_error="
                          << max_error << "deg elapsed=" << elapsed_sec
                          << "s" << (timed_out ? " timed_out" : "") << '\n';
                velocity.fill(0.0f);
            }

            {
                std::lock_guard<std::mutex> lock(drfl_mutex_);
                drfl_.speedj_rt(
                    velocity.data(),
                    acceleration.data(),
                    command_snapshot.command_time_sec);
            }

            if (target_done && command_snapshot.stop_rt_when_done)
            {
                std::lock_guard<std::mutex> command_lock(command_mutex_);
                command_.mode = CommandMode::Hold;
                command_.trajectory_active = false;
                command_.stop_rt_when_done = false;
                command_.velocity_target_command = velocity;
                command_.velocity_target_last_update = now;
                command_.position = current_position;
                realtime_loop_running_ = false;
            }
            else
            {
                std::lock_guard<std::mutex> command_lock(command_mutex_);
                if (command_.mode == CommandMode::JointVelocityTarget)
                {
                    command_.velocity_target_command = velocity;
                    command_.velocity_target_last_update = now;
                    command_.position = current_position;
                }
            }
        }

        std::this_thread::sleep_until(next);
    }

    stopRealtimeSession();
}

void DoosanController::handleAccessControl(MONITORING_ACCESS_CONTROL access)
{
    std::cout << "[callback] access control: " << accessControlToString(access) << '\n';

    if (access == MONITORING_ACCESS_CONTROL_GRANT)
    {
        has_control_access_ = true;
        is_standby_ = false;
    }
    else if (access == MONITORING_ACCESS_CONTROL_DENY ||
             access == MONITORING_ACCESS_CONTROL_LOSS)
    {
        has_control_access_ = false;
        is_standby_ = false;
    }
}

void DoosanController::handleRobotState(ROBOT_STATE state)
{
    std::cout << "[callback] robot state: " << robotStateToString(state) << '\n';
    is_standby_ = state == STATE_STANDBY;
}

void DoosanController::handleDisconnected()
{
    std::cerr << "[callback] disconnected from robot\n";
    disconnected_ = true;
    connected_ = false;
    servo_on_ = false;
    realtime_loop_running_ = false;
    realtime_control_running_ = false;
    realtime_control_connected_ = false;
}

void DoosanController::onAccessControl(MONITORING_ACCESS_CONTROL access)
{
    if (active_instance_)
    {
        active_instance_->handleAccessControl(access);
    }
}

void DoosanController::onRobotState(ROBOT_STATE state)
{
    if (active_instance_)
    {
        active_instance_->handleRobotState(state);
    }
}

void DoosanController::onDisconnected()
{
    if (active_instance_)
    {
        active_instance_->handleDisconnected();
    }
}

const char *DoosanController::robotStateToString(ROBOT_STATE state)
{
    switch (state)
    {
    case STATE_INITIALIZING: return "INITIALIZING";
    case STATE_STANDBY: return "STANDBY";
    case STATE_MOVING: return "MOVING";
    case STATE_SAFE_OFF: return "SAFE_OFF";
    case STATE_TEACHING: return "TEACHING";
    case STATE_SAFE_STOP: return "SAFE_STOP";
    case STATE_EMERGENCY_STOP: return "EMERGENCY_STOP";
    case STATE_HOMMING: return "HOMMING";
    case STATE_RECOVERY: return "RECOVERY";
    case STATE_SAFE_STOP2: return "SAFE_STOP2";
    case STATE_SAFE_OFF2: return "SAFE_OFF2";
    case STATE_NOT_READY: return "NOT_READY";
    default: return "UNKNOWN";
    }
}

const char *DoosanController::accessControlToString(MONITORING_ACCESS_CONTROL access)
{
    switch (access)
    {
    case MONITORING_ACCESS_CONTROL_REQUEST: return "REQUEST";
    case MONITORING_ACCESS_CONTROL_GRANT: return "GRANT";
    case MONITORING_ACCESS_CONTROL_DENY: return "DENY";
    case MONITORING_ACCESS_CONTROL_LOSS: return "LOSS";
    default: return "UNKNOWN";
    }
}

const char *DoosanController::robotSystemToString(RobotSystem system)
{
    return system == RobotSystem::Real ? "real" : "virtual";
}

}  // namespace doosan
