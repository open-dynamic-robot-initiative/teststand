/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2021, New York University and Max Planck Gesellschaft
 * 
 * @brief  Abstract interface to the Teststand robot.
 */

#ifndef TeststandAbstractInterface_H
#define TeststandAbstractInterface_H

#include <Eigen/Eigen>

namespace teststand
{
/**
 * @brief The class TeststandAbstractInterface is used to define an interface
 * to the Teststand robot dirvers.
 * The robots are composed of a single leg on a vertical rail and a TI force
 * torque sensors and a height sensor.
 */
class TeststandAbstractInterface
{
public:
    /**
     * @brief This represents the contact sensor
     */
    typedef Eigen::Matrix<double, 1, 1> VectorContact;
    /**
     * @brief This type define the forces from the ati-FT sensors
     */
    typedef Eigen::Matrix<double, 3, 1> VectorAtiForce;
    /**
     * @brief This type define the torques from the ati-FT sensors
     */
    typedef Eigen::Matrix<double, 3, 1> VectorAtiTorque;

    /**
     * @brief TeststandAbstractInterface is the constructor of the class.
     */
    TeststandAbstractInterface()
    {
        motor_inertias_.fill(0.0);
        motor_torque_constants_.fill(0.0);
        joint_positions_.fill(0.0);
        joint_velocities_.fill(0.0);
        joint_torques_.fill(0.0);
        joint_target_torques_.fill(0.0);
        joint_gear_ratios_.fill(0.0);
        joint_encoder_index_.fill(0.0);
        joint_zero_positions_.fill(0.0);
        slider_positions_.fill(0.0);
        contact_sensors_states_.fill(0.0);
        height_sensors_states_.fill(0.0);
        motor_max_current_.fill(0.0);
        motor_enabled_.fill(false);
        motor_ready_.fill(false);
        motor_board_enabled_.fill(false);
        motor_board_errors_.fill(0);
        ati_force_.fill(0.0);
        ati_torque_.fill(0.0);
    }

    /**
     * @brief Destroy the Teststand Abstract Interface object.
     */
    virtual ~TeststandAbstractInterface()
    {
    }

    /**
     * @brief initialize the robot by setting aligning the motors and calibrate
     * the sensors to 0
     */
    virtual void initialize() = 0;

    /**
     * @brief send_target_torques sends the target currents to the motors
     */
    virtual void send_target_joint_torque(
        const Eigen::Ref<const Eigen::Vector2d> target_joint_torque) = 0;

    /**
     * @brief acquire_sensors acquire all available sensors, WARNING !!!!
     * this method has to be called prior to any getter to have up to date data.
     */
    virtual bool acquire_sensors() = 0;

    /**
     * @brief Set max current.
     *
     * @param max_current
     */
    virtual void set_max_current(double max_current) = 0;

    /**
     * @brief This function will run a small controller that will move the
     * joints until the next joint index and reset the joint zero with this
     * knowledge.
     *
     * @return true if success
     * @return false if failure
     */
    virtual void calibrate(const Eigen::Vector2d& home_offset_rad) = 0;

    /**
     * @brief get_motor_inertias
     * @return the motor inertias
     */
    const Eigen::Ref<Eigen::Vector2d> get_motor_inertias()
    {
        return motor_inertias_;
    }

    /**
     * @brief get_motor_torque_constants
     * @return the torque constants of each motor
     */
    const Eigen::Ref<Eigen::Vector2d> get_motor_torque_constants()
    {
        return motor_torque_constants_;
    }

    /**
     * @brief get_joint_positions
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     *
     * @return  the joint angle of each module
     */
    const Eigen::Ref<Eigen::Vector2d> get_joint_positions()
    {
        return joint_positions_;
    }

    /**
     * @brief get_joint_velocities
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     *
     * @return the joint velocities
     */
    const Eigen::Ref<Eigen::Vector2d> get_joint_velocities()
    {
        return joint_velocities_;
    }

    /**
     * @brief get_joint_torques
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     *
     * @return the joint torques
     */
    const Eigen::Ref<Eigen::Vector2d> get_joint_torques()
    {
        return joint_torques_;
    }

    /**
     * @brief get_joint_torques
     * @return the target joint torques
     */
    const Eigen::Ref<Eigen::Vector2d> get_joint_target_torques()
    {
        return joint_target_torques_;
    }

    /**
     * @brief get_joint_gear_ratios
     * @return  the joint gear ratios
     */
    const Eigen::Ref<Eigen::Vector2d> get_joint_gear_ratios()
    {
        return joint_gear_ratios_;
    }

    /**
     * @brief get_joint_encoder_index
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     *
     * @return The last observed encoder index in joint coordinates.
     */
    const Eigen::Ref<Eigen::Vector2d> get_joint_encoder_index()
    {
        return joint_encoder_index_;
    }

    /**
     * @brief get_zero_positions
     * @return the position where the robot should be in "zero" configuration
     */
    const Eigen::Ref<Eigen::Vector2d> get_zero_positions()
    {
        return joint_zero_positions_;
    }

    /**
     * @brief get_slider_positions
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     *
     * @return the current sliders positions.
     */
    const Eigen::Ref<Eigen::Vector2d> get_slider_positions()
    {
        return slider_positions_;
    }

    /**
     * @brief get_contact_sensors_states
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     *
     * @return the state of the contacts states
     */
    const Eigen::Ref<VectorContact> get_contact_sensors_states()
    {
        return contact_sensors_states_;
    }

    /**
     * @brief get_contact_sensors_states
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     *
     * @return the state of the contacts states
     */
    const Eigen::Ref<VectorContact> get_height_sensors()
    {
        return height_sensors_states_;
    }

    /**
     * @brief get_max_current
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     * @return the max current.
     */
    const Eigen::Ref<Eigen::Vector2d> get_max_current()
    {
        return motor_max_current_;
    }

    /**
     * @brief Get motor enabled.
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     * @return const Eigen::Matrix<bool, 4>&
     */
    const Eigen::Matrix<bool, 2, 1>& get_motor_enabled() const
    {
        return motor_enabled_;
    }

    /**
     * @brief Get motor ready.
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     * @return const Eigen::Matrix<bool, 1, 1>&
     */
    const Eigen::Matrix<bool, 2, 1>& get_motor_ready() const
    {
        return motor_ready_;
    }

    /**
     * @brief Get motor board enabled.
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     * @return const Eigen::Matrix<bool, 1, 1>&
     */
    const Eigen::Matrix<bool, 1, 1>& get_motor_board_enabled() const
    {
        return motor_board_enabled_;
    }

    /**
     * @brief Get motor board errors.
     * @return const Eigen::Matrix<int, 2>&
     */
    const Eigen::Matrix<int, 1, 1>& get_motor_board_errors() const
    {
        return motor_board_errors_;
    }

    /**
     * @brief Get the ati_force_ object
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     *
     * @return const Eigen::Ref<VectorAtiForce>
     */
    const Eigen::Ref<VectorAtiForce> get_ati_force()
    {
        return ati_force_;
    }

    /**
     * @brief Get the ati_torque_ object
     * WARNING !!!! The method acquire_sensors() has to be called prior to
     * any getter to have up to date data.
     *
     * @return const Eigen::Ref<VectorAtiTorque>
     */
    const Eigen::Ref<VectorAtiTorque> get_ati_torque()
    {
        return ati_torque_;
    }

protected:
    /**
     * Motor data
     */

    /**
     * @brief motor_inertias_
     */
    Eigen::Vector2d motor_inertias_;
    /**
     * @brief motor_torque_constants_ are the motor torque constants
     */
    Eigen::Vector2d motor_torque_constants_;

    /**
     * Joint data
     */

    /**
     * @brief joint_positions_ is the measured data from the onboard card
     * converted at the joint level.
     */
    Eigen::Vector2d joint_positions_;
    /**
     * @brief joint_velocities_ is the measured data from the onboard card
     * converted at the joint level.
     */
    Eigen::Vector2d joint_velocities_;
    /**
     * @brief joint_torques_ is the measured data from the onboard card
     * converted at the joint level.
     */
    Eigen::Vector2d joint_torques_;
    /**
     * @brief joint_target_torques_ is the last given command to be sent.
     */
    Eigen::Vector2d joint_target_torques_;
    /**
     * @brief joint_gear_ratios are the joint gear ratios
     */
    Eigen::Vector2d joint_gear_ratios_;
    /**
     * @brief joint_encoder_index_ The last observed encoder_index at the
     * joints.
     */
    Eigen::Vector2d joint_encoder_index_;

    /**
     * @brief joint_zero_positions_ is the configuration considered as zero
     * position
     */
    Eigen::Vector2d joint_zero_positions_;

    /**
     * Additional data
     */

    /**
     * @brief slider_positions_ is the position of the linear potentiometer.
     * Can be used as a joystick input.
     */
    Eigen::Vector2d slider_positions_;

    /**
     * @brief contact_sensors_ is contact sensors at the foot
     */
    VectorContact contact_sensors_states_;

    /**
     * @brief height_sensors_ is the height position of the base.
     */
    VectorContact height_sensors_states_;

    /**
     * @brief max_current_ is the maximum current that can be sent to the
     * motors, this a safe guard for development
     */
    Eigen::Vector2d motor_max_current_;

    /**
     * @brief This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    Eigen::Matrix<bool, 2, 1 > motor_enabled_;

    /**
     * @brief This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    Eigen::Matrix<bool, 2, 1 > motor_ready_;

    /**
     * @brief This gives the status (enabled/disabled of the onboard control
     * cards)
     */
    Eigen::Matrix<bool, 1, 1> motor_board_enabled_;

    /**
     * @brief This gives the status (enabled/disabled of the onboard control
     * cards)
     */
    Eigen::Matrix<int, 1, 1> motor_board_errors_;

    /**
     * @brief 3D linear force from the ATI FT sensor
     */
    VectorAtiForce ati_force_;

    /**
     * @brief 3D torque measured from the ATI FT sensor
     */
    VectorAtiTorque ati_torque_;
};

}  // namespace teststand

#endif  // TeststandAbstractInterface_H
