/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2021, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implement the Teststand class.
 */

#include "odri_control_interface/utils.hpp"
#include "real_time_tools/timer.hpp"
#include "teststand/teststand.hpp"

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

    active_estop_ = false;
}

void Teststand::set_max_current(double max_current)
{
    robot_->joints->SetMaximumCurrents(max_current);
}

void Teststand::initialize(const std::string& network_id)
{
    // Use a serial port to read slider values.
    serial_reader_ =
        std::make_shared<slider_box::SerialReader>("/dev/not_used", 5);

    // Main driver interface.
    robot_ = odri_control_interface::RobotFromYamlFile(
        network_id, ODRI_CONTROL_INTERFACE_YAML_PATH);

    calib_ctrl_ = odri_control_interface::JointCalibratorFromYamlFile(
        ODRI_CONTROL_INTERFACE_YAML_PATH, robot_->joints);

    // Initialize the robot.
    robot_->Init();
}

void Teststand::wait_until_ready()
{
    robot_->WaitUntilReady();
}

bool Teststand::acquire_sensors()
{
    static int estop_counter_ = 0;

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

    height_sensors_states_(0) =
                1.0701053378814493 - 0.017 -
                1.0275690598232334 *
                robot_->robot_if->motor_drivers[0].adc[0];

    // acquire the slider positions
    serial_reader_->fill_vector(slider_positions_vector_);
    for (unsigned i = 0; i < slider_positions_.size(); ++i)
    {
        // acquire the slider
        slider_positions_(i) = double(slider_positions_vector_[i + 1]) / 1024.;
    }

    // Active the estop if button is pressed or the estop was active before.
    active_estop_ |= slider_positions_vector_[0] == 0;

    if (active_estop_ && estop_counter_++ % 2000 == 0)
    {
        robot_->ReportError("Soft E-Stop is active.");
    }

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

    Eigen::Vector2d target;
    target.setZero();
    robot_->RunCalibration(calib_ctrl_, target);
}

void Teststand::calibrate()
{
    Eigen::Vector2d target;
    target.setZero();
    robot_->RunCalibration(calib_ctrl_, target);
}

}  // namespace teststand
