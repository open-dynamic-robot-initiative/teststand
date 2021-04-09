/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2021, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implement the Teststand class.
 */

#include "teststand/teststand.hpp"

#include <cmath>

#include "odri_control_interface/utils.hpp"

namespace teststand
{
Teststand::Teststand()
{
    /**
     * Additional data
     */

    // imu infos
    base_accelerometer_.setZero();
    base_gyroscope_.setZero();
    base_attitude_.setZero();
    base_linear_acceleration_.setZero();
    base_attitude_quaternion_.setZero();

    // Finite state machine for the control
    control_state_ = ControlState::initial;
    calibrate_request_ = false;
    nb_time_we_acquired_sensors_ = 0;
}

void Teststand::set_max_current(double max_current)
{
    robot_->joints->SetMaximumCurrents(max_current);
}

void Teststand::initialize(const std::string& network_id)
{
    // Main driver interface.
    robot_ = odri_control_interface::RobotFromYamlFile(
        network_id, ODRI_CONTROL_INTERFACE_YAML_PATH);

    calib_ctrl_ = odri_control_interface::JointCalibratorFromYamlFile(
        ODRI_CONTROL_INTERFACE_YAML_PATH, robot_->joints);

    // Use a serial port to read slider values.
    serial_reader_ = std::make_shared<blmc_drivers::SerialReader>(
        "serial_port", TESTSTAND_NB_SLIDER + 1);

    // Initialize the robot.
    robot_->Init();
}

void Teststand::wait_until_ready()
{
    robot_->WaitUntilReady();
}

bool Teststand::acquire_sensors()
{
    // Acquire the data.
    robot_->ParseSensorData();

    /**
     * Joint data
     */
    // acquire the joint position
    joint_positions_ = robot_->joints->GetPositions();
    // acquire the joint velocities
    joint_velocities_ = robot_->joints->GetVelocities();
    // acquire the joint torques
    joint_torques_ = robot_->joints->GetMeasuredTorques();
    // acquire the target joint torques
    joint_target_torques_ = robot_->joints->GetSentTorques();

    /**
     * Additional data
     */
    base_accelerometer_ = robot_->imu->GetAccelerometer();
    base_gyroscope_ = robot_->imu->GetGyroscope();
    base_attitude_ = robot_->imu->GetAttitudeEuler();
    base_attitude_quaternion_ = robot_->imu->GetAttitudeQuaternion();
    base_linear_acceleration_ = robot_->imu->GetLinearAcceleration();

    // acquire the slider positions
    // if (serial_reader_->fill_vector(slider_box_data_) > 10)
    // {
    //     robot_->ReportError();
    //     if(nb_time_we_acquired_sensors_ % 2000 == 0)
    //     {
    //         robot_->ReportError(
    //         "The slider box is not responding correctly, "
    //         "10 iteration are missing.");
    //     }
    // }
    // for (unsigned i = 0; i < slider_positions_.size(); ++i)
    // {
    //     // acquire the slider
    //     slider_positions_(i) = double(slider_box_data_[i + 1]) / 1024.;
    // }
    // // acquire the e-stop from the slider box
    // active_estop_ = slider_box_data_[0] == 0;

    /**
     * The different status.
     */

    // motor board status
    motor_board_enabled_[0] = robot_->joints->GetMotorDriverEnabled()[0];
    motor_board_errors_[0] = robot_->joints->GetMotorDriverErrors()[0];

    // motors status
    motor_enabled_[0] = robot_->joints->GetEnabled()[0];
    motor_enabled_[1] = robot_->joints->GetEnabled()[1];
    motor_ready_[0] = robot_->joints->GetReady()[0];
    motor_ready_[1] = robot_->joints->GetReady()[1];

    ++nb_time_we_acquired_sensors_;

    return true;
}

void Teststand::send_target_joint_torque(
    const Eigen::Ref<const Eigen::Vector2d> target_joint_torque)
{
    robot_->joints->SetTorques(target_joint_torque);

    switch (control_state_)
    {
        case ControlState::initial:
            robot_->joints->SetZeroCommands();
            if (!robot_->IsTimeout() && !robot_->IsAckMsgReceived())
            {
                robot_->SendInit();
            }
            else if (!robot_->IsReady())
            {
                robot_->SendCommand();
            }
            else
            {
                control_state_ = ControlState::ready;
                robot_->SendCommand();
            }
            break;

        case ControlState::ready:
            if (calibrate_request_)
            {
                calibrate_request_ = false;
                control_state_ = ControlState::calibrate;
                robot_->joints->SetZeroCommands();
            }
            robot_->SendCommand();
            break;

        case ControlState::calibrate:
            // calib_ctrl_ set the robot_->joints torque commands;
            if (calib_ctrl_->Run())
            {
                control_state_ = ControlState::ready;
            }
            robot_->SendCommand();
            break;
    }
}

void Teststand::request_calibration(
    const Eigen::Ref<const Eigen::VectorXd> home_offset_rad)
{
    printf("Teststand::calibrate called\n");
    calib_ctrl_->UpdatePositionOffsets(home_offset_rad);
    calibrate_request_ = true;
}

void Teststand::request_calibration()
{
    printf("Teststand::calibrate called\n");
    calibrate_request_ = true;
}

void Teststand::calibrate(const Eigen::Vector2d& home_offset_rad)
{
    calib_ctrl_->UpdatePositionOffsets(home_offset_rad);
    robot_->RunCalibration(calib_ctrl_);
}

}  // namespace teststand
