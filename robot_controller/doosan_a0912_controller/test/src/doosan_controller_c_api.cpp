#include "doosan_controller.hpp"

#include <array>
#include <algorithm>
#include <chrono>
#include <exception>
#include <iostream>
#include <string>

namespace
{
using Controller = doosan::DoosanController;
using JointArray = Controller::JointArray;

Controller *asController(void *handle)
{
    return static_cast<Controller *>(handle);
}

JointArray toJointArray(const float *values)
{
    JointArray out = {};
    if (!values)
    {
        return out;
    }

    for (int i = 0; i < Controller::kNumJoints; ++i)
    {
        out[i] = values[i];
    }
    return out;
}

}  // namespace

extern "C"
{

void *doosan_controller_create()
{
    try
    {
        return new Controller();
    }
    catch (const std::exception &e)
    {
        std::cerr << "[c_api] create failed: " << e.what() << '\n';
    }
    catch (...)
    {
        std::cerr << "[c_api] create failed with unknown exception\n";
    }
    return nullptr;
}

void doosan_controller_destroy(void *handle)
{
    try
    {
        delete asController(handle);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[c_api] destroy failed: " << e.what() << '\n';
    }
    catch (...)
    {
        std::cerr << "[c_api] destroy failed with unknown exception\n";
    }
}

int doosan_controller_initialize(
    void *handle,
    const char *ip,
    unsigned int port,
    const char *mode,
    const char *rt_ip,
    unsigned int rt_port,
    int enable_realtime_control,
    int timeout_sec)
{
    try
    {
        auto *controller = asController(handle);
        if (!controller)
        {
            return 0;
        }

        Controller::Config config;
        if (ip && std::string(ip).size() > 0)
        {
            config.ip = ip;
        }
        config.port = port;
        if (mode && std::string(mode).size() > 0)
        {
            config.system = Controller::robotSystemFromString(mode);
        }
        if (rt_ip && std::string(rt_ip).size() > 0)
        {
            config.rt_ip = rt_ip;
        }
        config.rt_port = rt_port;

        return controller->initialize(
                   config,
                   enable_realtime_control != 0,
                   std::chrono::seconds(timeout_sec > 0 ? timeout_sec : 15))
                   ? 1
                   : 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[c_api] initialize failed: " << e.what() << '\n';
    }
    catch (...)
    {
        std::cerr << "[c_api] initialize failed with unknown exception\n";
    }
    return 0;
}

// Queries return 1 on success, 0 on failure. Output buffers are unchanged on failure.
int doosan_controller_read_actual_joint_state(
    void *handle, float *position, float *velocity, int *motion_running)
{
    try
    {
        auto *controller = asController(handle);
        if (!controller || !position || !velocity || !motion_running)
        {
            return 0;
        }
        JointArray measured_position = {}, measured_velocity = {};
        bool running = false;
        if (!controller->readActualJointState(&measured_position, &measured_velocity, &running))
        {
            return 0;
        }
        std::copy(measured_position.begin(), measured_position.end(), position);
        std::copy(measured_velocity.begin(), measured_velocity.end(), velocity);
        *motion_running = running ? 1 : 0;
        return 1;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[c_api] read_actual_joint_state failed: " << e.what() << '\n';
    }
    catch (...)
    {
        std::cerr << "[c_api] read_actual_joint_state failed with unknown exception\n";
    }
    return 0;
}

int doosan_controller_read_actual_cartesian_state(void *handle, float *flange_pose, float *tcp_pose)
{
    try
    {
        auto *controller = asController(handle);
        if (!controller || !flange_pose || !tcp_pose)
        {
            return 0;
        }
        JointArray flange = {}, tcp = {};
        if (!controller->readActualCartesianState(&flange, &tcp))
        {
            return 0;
        }
        std::copy(flange.begin(), flange.end(), flange_pose);
        std::copy(tcp.begin(), tcp.end(), tcp_pose);
        return 1;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[c_api] read_actual_cartesian_state failed: " << e.what() << '\n';
    }
    catch (...)
    {
        std::cerr << "[c_api] read_actual_cartesian_state failed with unknown exception\n";
    }
    return 0;
}

// 1 = solution, 0 = SDK reports no usable solution (status 1/2), -1 = query error.
int doosan_controller_solve_closest_ik(
    void *handle, const float *task_pose, const float *reference_joint,
    float *joint, int *solution_space, int *sdk_status)
{
    if (sdk_status)
    {
        *sdk_status = -1;
    }
    try
    {
        auto *controller = asController(handle);
        if (!controller || !task_pose || !reference_joint || !joint || !solution_space || !sdk_status)
        {
            return -1;
        }
        JointArray result = {};
        int solution = -1, status = -1;
        const bool ok = controller->solveClosestIk(
            toJointArray(task_pose), toJointArray(reference_joint), &result, &solution, &status);
        *sdk_status = status;
        if (!ok)
        {
            return status == 1 || status == 2 ? 0 : -1;
        }
        std::copy(result.begin(), result.end(), joint);
        *solution_space = solution;
        return 1;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[c_api] solve_closest_ik failed: " << e.what() << '\n';
    }
    catch (...)
    {
        std::cerr << "[c_api] solve_closest_ik failed with unknown exception\n";
    }
    return -1;
}

int doosan_controller_position_control(
    void *handle,
    const float *position,
    const float *velocity,
    const float *acceleration,
    float command_time_sec)
{
    try
    {
        auto *controller = asController(handle);
        if (!controller)
        {
            return 0;
        }

        return controller->positionControl(
                   toJointArray(position),
                   toJointArray(velocity),
                   toJointArray(acceleration),
                   command_time_sec)
                   ? 1
                   : 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[c_api] position_control failed: " << e.what() << '\n';
    }
    catch (...)
    {
        std::cerr << "[c_api] position_control failed with unknown exception\n";
    }
    return 0;
}

int doosan_controller_position_control_timed(
    void *handle,
    const float *position,
    const float *velocity,
    const float *acceleration,
    float move_time_sec,
    float command_time_sec)
{
    try
    {
        auto *controller = asController(handle);
        if (!controller)
        {
            return 0;
        }

        return controller->positionControl(
                   toJointArray(position),
                   toJointArray(velocity),
                   toJointArray(acceleration),
                   move_time_sec,
                   command_time_sec)
                   ? 1
                   : 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[c_api] position_control_timed failed: " << e.what() << '\n';
    }
    catch (...)
    {
        std::cerr << "[c_api] position_control_timed failed with unknown exception\n";
    }
    return 0;
}

int doosan_controller_velocity_control(
    void *handle,
    const float *velocity,
    const float *acceleration,
    float command_time_sec)
{
    try
    {
        auto *controller = asController(handle);
        if (!controller)
        {
            return 0;
        }

        return controller->velocityControl(
                   toJointArray(velocity),
                   toJointArray(acceleration),
                   command_time_sec)
                   ? 1
                   : 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[c_api] velocity_control failed: " << e.what() << '\n';
    }
    catch (...)
    {
        std::cerr << "[c_api] velocity_control failed with unknown exception\n";
    }
    return 0;
}

int doosan_controller_velocity_control_to_position(
    void *handle,
    const float *target_position,
    const float *max_velocity,
    const float *acceleration,
    float gain,
    float tolerance_deg,
    float timeout_sec,
    float command_time_sec)
{
    try
    {
        auto *controller = asController(handle);
        if (!controller)
        {
            return 0;
        }

        return controller->velocityControlToPosition(
                   toJointArray(target_position),
                   toJointArray(max_velocity),
                   toJointArray(acceleration),
                   gain,
                   tolerance_deg,
                   timeout_sec,
                   command_time_sec)
                   ? 1
                   : 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[c_api] velocity_control_to_position failed: " << e.what() << '\n';
    }
    catch (...)
    {
        std::cerr << "[c_api] velocity_control_to_position failed with unknown exception\n";
    }
    return 0;
}

int doosan_controller_task_trajectory_csv_position_control(
    void *handle,
    const char *csv_path,
    float command_time_sec,
    float max_joint_step_deg,
    float max_joint_velocity_deg_s,
    float max_joint_acceleration_deg_s2)
{
    try
    {
        auto *controller = asController(handle);
        if (!controller || !csv_path)
        {
            return 0;
        }

        return controller->taskTrajectoryCsvPositionControl(
                   csv_path,
                   command_time_sec,
                   max_joint_step_deg,
                   max_joint_velocity_deg_s,
                   max_joint_acceleration_deg_s2)
                   ? 1
                   : 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[c_api] task_trajectory_csv_position_control failed: "
                  << e.what() << '\n';
    }
    catch (...)
    {
        std::cerr << "[c_api] task_trajectory_csv_position_control failed with unknown exception\n";
    }
    return 0;
}

int doosan_controller_joint_trajectory_csv_position_control(
    void *handle,
    const char *csv_path,
    float command_time_sec,
    float max_joint_step_deg,
    float max_joint_velocity_deg_s,
    float max_joint_acceleration_deg_s2)
{
    try
    {
        auto *controller = asController(handle);
        if (!controller || !csv_path)
        {
            return 0;
        }

        return controller->jointTrajectoryCsvPositionControl(
                   csv_path,
                   command_time_sec,
                   max_joint_step_deg,
                   max_joint_velocity_deg_s,
                   max_joint_acceleration_deg_s2)
                   ? 1
                   : 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[c_api] joint_trajectory_csv_position_control failed: "
                  << e.what() << '\n';
    }
    catch (...)
    {
        std::cerr << "[c_api] joint_trajectory_csv_position_control failed with unknown exception\n";
    }
    return 0;
}

int doosan_controller_joint_trajectory_csv_velocity_control(
    void *handle,
    const char *csv_path,
    float command_time_sec,
    float position_gain,
    float max_joint_step_deg,
    float max_joint_velocity_deg_s,
    float max_joint_acceleration_deg_s2,
    float preposition_max_joint_velocity_deg_s,
    float preposition_max_joint_acceleration_deg_s2)
{
    try
    {
        auto *controller = asController(handle);
        if (!controller || !csv_path)
        {
            return 0;
        }

        return controller->jointTrajectoryCsvVelocityControl(
                   csv_path,
                   command_time_sec,
                   position_gain,
                   max_joint_step_deg,
                   max_joint_velocity_deg_s,
                   max_joint_acceleration_deg_s2,
                   preposition_max_joint_velocity_deg_s,
                   preposition_max_joint_acceleration_deg_s2)
                   ? 1
                   : 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[c_api] joint_trajectory_csv_velocity_control failed: "
                  << e.what() << '\n';
    }
    catch (...)
    {
        std::cerr << "[c_api] joint_trajectory_csv_velocity_control failed with unknown exception\n";
    }
    return 0;
}

int doosan_controller_task_trajectory_csv_velocity_control(
    void *handle,
    const char *csv_path,
    const char *urdf_path,
    const char *tcp_frame,
    float command_time_sec,
    float position_gain,
    float orientation_gain,
    float damping,
    float max_joint_velocity_deg_s,
    float max_joint_acceleration_deg_s2,
    float max_joint_step_deg)
{
    try
    {
        auto *controller = asController(handle);
        if (!controller || !csv_path || !urdf_path)
        {
            return 0;
        }

        return controller->taskTrajectoryCsvVelocityControl(
                   csv_path,
                   urdf_path,
                   tcp_frame ? tcp_frame : "link_6",
                   command_time_sec,
                   position_gain,
                   orientation_gain,
                   damping,
                   max_joint_velocity_deg_s,
                   max_joint_acceleration_deg_s2,
                   max_joint_step_deg)
                   ? 1
                   : 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[c_api] task_trajectory_csv_velocity_control failed: "
                  << e.what() << '\n';
    }
    catch (...)
    {
        std::cerr << "[c_api] task_trajectory_csv_velocity_control failed with unknown exception\n";
    }
    return 0;
}

void doosan_controller_hold(void *handle)
{
    try
    {
        if (auto *controller = asController(handle))
        {
            controller->holdCommand();
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "[c_api] hold failed: " << e.what() << '\n';
    }
    catch (...)
    {
        std::cerr << "[c_api] hold failed with unknown exception\n";
    }
}

int doosan_controller_is_realtime_control_running(void *handle)
{
    try
    {
        auto *controller = asController(handle);
        return controller && controller->isRealtimeControlRunning() ? 1 : 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[c_api] is_realtime_control_running failed: " << e.what() << '\n';
    }
    catch (...)
    {
        std::cerr << "[c_api] is_realtime_control_running failed with unknown exception\n";
    }
    return 0;
}

int doosan_controller_servo_off(void *handle)
{
    try
    {
        auto *controller = asController(handle);
        return controller ? (controller->servoOff() ? 1 : 0) : 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[c_api] servo_off failed: " << e.what() << '\n';
    }
    catch (...)
    {
        std::cerr << "[c_api] servo_off failed with unknown exception\n";
    }
    return 0;
}

void doosan_controller_shutdown(void *handle)
{
    try
    {
        if (auto *controller = asController(handle))
        {
            controller->shutdown();
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "[c_api] shutdown failed: " << e.what() << '\n';
    }
    catch (...)
    {
        std::cerr << "[c_api] shutdown failed with unknown exception\n";
    }
}

}  // extern "C"
