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

    // 0 torques
    Eigen::Vector2d desired_torque = Eigen::Vector2d::Zero();

    rt_printf("Sensor reading loop finished waiting \n");

    size_t count = 0;
    while (!CTRL_C_DETECTED)
    {
        // acquire the sensors
        robot.acquire_sensors();

        // print -----------------------------------------------------------
        if ((count % 1000) == 0)
        {
            rt_printf("\33[H\33[2J");  // clear screen
            rt_printf("Sensory data:");
            rt_printf("\n");
            robot.print_all();
            print_vector("slider_positions:       ", robot.get_slider_positions());
            rt_printf("\n");
            fflush(stdout);
        }
        ++count;

        // Send the current to the motor
        // desired_torque.setZero();
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

    rt_printf("sensor reader is set up \n");
    thread.create_realtime_thread(&control_loop, &robot);
    while (!CTRL_C_DETECTED)
    {
        real_time_tools::Timer::sleep_sec(0.001);
    }

    thread.join();

    return 0;
}
