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
bool print_home_offset = false;

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_void_ptr)
{
    Teststand& robot = *(static_cast<Teststand*>(robot_void_ptr));

    Eigen::Vector2d zero_torques = Eigen::Vector2d::Zero();

    long int counter = 0;

    real_time_tools::Spinner spinner;
    spinner.set_period(0.001);
    
    while (!CTRL_C_DETECTED)
    {
        robot.acquire_sensors();

        if (counter++ % 200 == 0 && print_home_offset)
        {
            print_vector("Home offset angle [Rad]", -robot.get_joint_positions());
        }

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

    thread.create_realtime_thread(&control_loop, &robot);

    rt_printf("Controller is set up.\n");
    
    rt_printf("Press enter to launch the calibration.\n");
    char str[256];
    std::cin.get(str, 256);  // get c-string

    Eigen::Vector2d zero_pos = Eigen::Vector2d::Zero();
    robot.request_calibration(zero_pos);

    print_home_offset = true;

    // Wait until the application is killed.
    thread.join();

    rt_printf("Exit cleanly.\n");

    return 0;
}
