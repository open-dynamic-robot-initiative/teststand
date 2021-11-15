/**
 * \file demo_bolt.cpp
 * \brief Implements basic PD controller reading slider values.
 * \author Julian Viereck
 * \date 21 November 2019
 *
 * This file uses the Solo12 class in a small demo.
 */

#include "teststand/teststand.hpp"
#include "teststand/utils.hpp"

using namespace teststand;

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* args)
{
    Teststand& robot = *static_cast<Teststand*>(args);

    double kp = 3.0;
    double kd = 0.05;
    double t = 0.0;
    double dt = 0.001;
    double freq = 0.3;
    double amplitude = M_PI / 8.0;
    Eigen::Vector2d desired_joint_position = Eigen::Vector2d::Zero();
    Eigen::Vector2d desired_torque = Eigen::Vector2d::Zero();

    Eigen::Vector2d init_pose;
    Eigen::Matrix<bool, 2, 1> motor_enabled;

    rt_printf("Calibrating... \n");
    robot.calibrate();

    rt_printf("Control loop started. \n");
    robot.acquire_sensors();
    Eigen::Vector2d initial_joint_positions = robot.get_joint_positions();
    size_t count = 0;
    while (!CTRL_C_DETECTED)
    {
        // acquire the sensors
        robot.acquire_sensors();

        // acquire the motor enabled signal.
        motor_enabled = robot.get_motor_enabled();

        // Desired pose and vel
        desired_joint_position =
            initial_joint_positions +
            Eigen::Vector2d::Ones() * amplitude * sin(2 * M_PI * freq * t);
        t += dt;

        // we implement here a small pd control at the current level
        desired_torque =
            kp * (desired_joint_position - robot.get_joint_positions()) -
            kd * robot.get_joint_velocities();

        // print -----------------------------------------------------------
        if ((count % 1000) == 0)
        {
            rt_printf("\33[H\33[2J");  // clear screen
            rt_printf("Sensory data:");
            rt_printf("\n");
            robot.print_all();
            rt_printf("\n");
            fflush(stdout);
        }
        ++count;

        // Send the current to the motor
        robot.send_target_joint_torque(desired_torque);

        real_time_tools::Timer::sleep_sec(0.001);
    }  // endwhile
    return THREAD_FUNCTION_RETURN_VALUE;
}  // end control_loop

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        throw std::runtime_error(
            "Wrong number of argument: `./demo_bolt network_id`.");
    }

    real_time_tools::RealTimeThread thread;
    enable_ctrl_c();

    Teststand robot;
    robot.initialize(std::string(argv[1]));
    thread.create_realtime_thread(&control_loop, &robot);

    while (!CTRL_C_DETECTED)
    {
        real_time_tools::Timer::sleep_sec(0.001);
    }

    thread.join();

    return 0;
}
