/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2021, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implementing the Teststand driver with the TI board and CAN protocol.
 */

#include <math.h>
#include "teststand/teststand_ti.hpp"

namespace teststand
{

TeststandTi::TeststandTi(): TeststandAbstractInterface()
{
    // ati_sensor_ nothing to be done.
    can_buses_.fill(nullptr);
    can_motor_boards_.fill(nullptr);
    motors_.fill(nullptr);
    // joints_ nothing to be done.
    height_sensors_.fill(nullptr);
}

void TeststandTi::initialize()
{
    // Initialize the internal data structures.
    motor_max_current_.fill(12); // Amper
    motor_torque_constants_.fill(0.025);
    motor_inertias_.fill(0.045);
    joint_gear_ratios_.fill(9.0);

    // initialize the communication with the can cards
    can_buses_[0] = std::make_shared<blmc_drivers::CanBus>("can0");
    can_motor_boards_[0] =
        std::make_shared<blmc_drivers::CanBusMotorBoard>(can_buses_[0]);

    // can 0.
    height_sensors_[0] =
        std::make_shared<blmc_drivers::AnalogSensor>(can_motor_boards_[0], 1);
    // motor_hfe.
    motors_[0] = std::make_shared<blmc_drivers::Motor>(can_motor_boards_[0], 1);
    // motor_kfe.
    motors_[1] = std::make_shared<blmc_drivers::Motor>(can_motor_boards_[0], 0);

    // Create the joint module objects.
    joints_.set_motor_array(motors_,
                            motor_torque_constants_,
                            joint_gear_ratios_,
                            joint_zero_positions_,
                            motor_max_current_);

    // The control gains in order to perform the calibration.
    Eigen::Vector2d kp, kd;
    kp.fill(2.0);
    kd.fill(0.05);
    joints_.set_position_control_gains(kp, kd);


    // wait until all board are ready and connected
    can_motor_boards_[0]->wait_until_ready();
    // can_motor_boards_[1]->wait_until_ready();
}

void TeststandTi::set_max_current(double)
{
    printf("set_max_current() not available with the TI drivers.");
}

bool TeststandTi::acquire_sensors()
{
    try
    {
        /**
         * Joint data
         */
        // acquire the joint position
        joint_positions_ = joints_.get_measured_angles();
        // acquire the joint velocities
        joint_velocities_ = joints_.get_measured_velocities();
        // acquire the joint torques
        joint_torques_ = joints_.get_measured_torques();
        // acquire the joint index
        joint_encoder_index_ = joints_.get_measured_index_angles();
        // acquire the target joint torques
        joint_target_torques_ = joints_.get_sent_torques();

        /**
         * Additional data
         */

        // acquire the height sensor.
        // Transforms the measurement into a height measurement between
        // - the center of the HFE joint
        // - and the mounting point **above** the table.
        height_sensors_states_(0) =
            1.0701053378814493 - 0.017 -
            1.0275690598232334 *
                height_sensors_[0]->get_measurement()->newest_element();
    }
    catch (std::exception ex)
    {
        rt_printf(
            "HARDWARE: Something went wrong during the sensor reading.\n");
        rt_printf("error is: %s\n", ex.what());
        return false;
    }
    return true;
}

void TeststandTi::send_target_joint_torque(
    const Eigen::Ref<const Eigen::Vector2d> target_joint_torque)
{
    Eigen::Vector2d ctrl_torque = target_joint_torque;
    ctrl_torque = ctrl_torque.array().min(joints_.get_max_torques().array());
    ctrl_torque = ctrl_torque.array().max(-joints_.get_max_torques().array());
    joints_.set_torques(ctrl_torque);
    joints_.send_torques();
}

void TeststandTi::calibrate(const Eigen::Vector2d& home_offset_rad)
{
    // Maximum distance is twice the angle between joint indexes
    double search_distance_limit_rad = 2.0 * (2.0 * M_PI / 9.0);
    Eigen::Vector2d profile_step_size_rad = Eigen::Vector2d::Constant(0.001);
    joints_.execute_homing(
        search_distance_limit_rad, home_offset_rad, profile_step_size_rad);
    Eigen::Vector2d zero_pose = Eigen::Vector2d::Zero();
    joints_.go_to(zero_pose);
}

}  // namespace teststand
