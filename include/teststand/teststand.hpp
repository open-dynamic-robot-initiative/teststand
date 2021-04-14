/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2021, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Teststand biped low level drivers.
 */

#pragma once

#include <math.h>

#include <Eigen/Eigen>

#include "blmc_drivers/serial_reader.hpp"
#include "odri_control_interface/calibration.hpp"
#include "odri_control_interface/robot.hpp"
#include "teststand/teststand_abstract_interface.hpp"

namespace teststand
{
#define TESTSTAND_NB_MOTOR_BOARD 1
#define TESTSTAND_NB_MOTOR 2
#define TESTSTAND_NB_SLIDER 0

/** @brief Control state of the robot. */
enum ControlState
{
    initial,
    ready,
    calibrate
};

/**
 * @brief Driver for the Teststand biped robot.
 */
class Teststand : public TeststandAbstractInterface
{
public:
    /**
     * @brief Teststand is the constructor of the class.
     */
    Teststand();

    virtual void initialize()
    {
        throw std::runtime_error(
            "Teststand::initialize: Missing the network id argument");
    }

    /**
     * @copydoc TeststandAbstractInterface::initialize
     */
    virtual void initialize(const std::string& network_id);

    /**
     * @copydoc TeststandAbstractInterface::send_target_joint_torque
     */
    virtual void send_target_joint_torque(
        const Eigen::Ref<const Eigen::Vector2d> target_joint_torque);

    /**
     * @copydoc TeststandAbstractInterface::acquire_sensors
     */
    virtual bool acquire_sensors();

    /**
     * @brief Set max current.
     */
    virtual void set_max_current(double max_current);

    /**
     * @copydoc TeststandAbstractInterface::calibrate
     */
    virtual void calibrate(const Eigen::Vector2d& home_offset_rad);

    /**
     * @brief Wait until the hardware is ready to be controlled.
     */
    void wait_until_ready();

    /**
     * @brief Request calibration of the joints by moving to the next joint
     * index position. The control is made inside the send_target_joint_torque.
     *
     * @param home_offset_rad This is the angle between the index and the zero
     * pose.
     * @return true
     * @return false
     */
    void request_calibration(
        const Eigen::Ref<const Eigen::VectorXd> home_offset_rad);

    /**
     * @brief Request calibration of the joints by moving to the next joint
     * index position. The control is made inside the send_target_joint_torque.
     * Use the yaml information in order to get the offset from the nearest
     * motor index.
     *
     * @return true
     * @return false
     */
    void request_calibration();

    /**
     * @brief get_base_accelerometer
     * @return the base_accelerometer
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<const Eigen::Vector3d> get_base_accelerometer()
    {
        return base_accelerometer_;
    }

    /**
     * @brief get_base_accelerometer
     * @return the base_accelerometer
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<const Eigen::Vector3d> get_base_gyroscope()
    {
        return base_gyroscope_;
    }

    /**
     * @brief get_base_accelerometer
     * @return the base_accelerometer
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<const Eigen::Vector3d> get_base_attitude()
    {
        return base_attitude_;
    }

    /**
     * @brief get_base_accelerometer
     * @return the base_accelerometer
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<const Eigen::Vector3d> get_base_linear_acceleration()
    {
        return base_linear_acceleration_;
    }

    /**
     * @brief get_base_accelerometer
     * @return the base_accelerometer
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<const Eigen::Vector4d> get_base_attitude_quaternion()
    {
        return base_attitude_quaternion_;
    }

    /**
     * @brief is_calibrating()
     * @return Returns true if the calibration procedure is running right now.
     */
    bool is_calibrating()
    {
        return (control_state_ == ControlState::calibrate) ||
               calibrate_request_;
    }

protected:
    /**
     * @brief Fill attitude quaternion.
     */
    void fill_base_attitude_quaternion();

private:
    /** @brief base accelerometer. */
    Eigen::Vector3d base_accelerometer_;

    /** @brief base accelerometer. */
    Eigen::Vector3d base_gyroscope_;

    /** @brief base accelerometer. */
    Eigen::Vector3d base_attitude_;

    /** @brief base accelerometer. */
    Eigen::Vector3d base_linear_acceleration_;

    /** @brief base attitude quaternion. */
    Eigen::Vector4d base_attitude_quaternion_;

    /*
     * Controllers
     */

    /** @brief Controller to run the calibration procedure */
    std::shared_ptr<odri_control_interface::JointCalibrator> calib_ctrl_;

    /*
     * Finite state machine of the controller.
     */

    /** @brief Control state Initial, Ready, Calibration */
    ControlState control_state_;

    /** @brief Check if the user called for the joint calibration. */
    bool calibrate_request_;

    /** @brief Number of time we acquire the sensor readings, this is used
     *  to prevent spamming prints */
    long int nb_time_we_acquired_sensors_;

    /*
     * Drivers communication objects
     */

    /**
     * @brief Robot complete interface.
     * Wrapper around the MasterBoardInterface.
     *
     * PC <------------------> main board <-------> Motor Board
     *       Ethernet/Wifi                   SPI
     */
    std::shared_ptr<odri_control_interface::Robot> robot_;

    /**
     * @brief Reader for serial port to read arduino slider values.
     */
    std::shared_ptr<blmc_drivers::SerialReader> serial_reader_;

    /**
     * @brief ATI sensor.
     */
    ati_ft_sensor::AtiFTSensor ati_sensor_;
};

}  // namespace teststand
