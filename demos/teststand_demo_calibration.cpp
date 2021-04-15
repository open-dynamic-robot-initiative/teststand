/**
 * @file demo_solo12_calibration.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief Small demo to test the calibration on the real robot.
 * @version 0.1
 * @date 2019-11-08
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "teststand/teststand.hpp"
#include "teststand/utils.hpp"

using namespace teststand;

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_void_ptr)
{
    Teststand& robot = *(static_cast<Teststand*>(robot_void_ptr));

    Eigen::Vector2d zero_torques = Eigen::Vector2d::Zero();

    robot.wait_until_ready();
    rt_printf("Running calibration...\n");
    robot.calibrate();
    rt_printf("Running calibration... Done.\n");

    rt_printf("Go idle indefinitely, ctrl+c to quit.\n");
    real_time_tools::Spinner spinner;
    spinner.set_period(0.001);
    while (!CTRL_C_DETECTED)
    {
        robot.acquire_sensors();
        robot.send_target_joint_torque(zero_torques);
        spinner.spin();
    }

    CTRL_C_DETECTED = true;
    return THREAD_FUNCTION_RETURN_VALUE;
}  // end control_loop

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        throw std::runtime_error(
            "Wrong number of argument: `sudo ./hardware_calibration "
            "network_id`.");
    }

    enable_ctrl_c();

    real_time_tools::RealTimeThread thread;

    Teststand robot;
    robot.initialize(argv[1]);

    rt_printf("Controller is set up.\n");
    
    rt_printf("Press enter to launch the calibration.\n");
    char str[256];
    std::cin.get(str, 256);  // get c-string

    thread.create_realtime_thread(&control_loop, &robot);

    // Wait until the application is killed.
    thread.join();

    rt_printf("Exit cleanly.\n");

    return 0;
}
