/**
 * \file teststandTi.hpp
 * \author Manuel Wuthrich
 * \date 2018
 * \copyright Copyright (c) 2019, New York University & Max Planck Gesellschaft.
 */

#ifndef TESTSTANDTi_H
#define TESTSTANDTi_H

#include "AtiFTSensor.h"
#include "blmc_drivers/blmc_joint_module.hpp"
#include "blmc_drivers/devices/analog_sensor.hpp"
#include "teststand/teststand_abstract_interface.hpp"

namespace teststand
{
/** @brief Shortcut for the shared pointer CanBus type. */
typedef std::shared_ptr<blmc_drivers::CanBus> CanBus_ptr;

/** @brief Shortcut for the shared pointer CanBus type. */
typedef std::shared_ptr<blmc_drivers::CanBusMotorBoard> CanBusMotorBoard_ptr;

/** @brief Shortcut for the shared pointer MotorInterface type. */
typedef std::shared_ptr<blmc_drivers::MotorInterface> MotorInterface_ptr;

/** @brief Shortcut for the contact sensor. It is also an analog sensor. */
typedef std::shared_ptr<blmc_drivers::AnalogSensor> ContactSensor_ptr;

/** @brief Shortcut for the height sensor. It is also an analog sensor. */
typedef std::shared_ptr<blmc_drivers::AnalogSensor> HeightSensor_ptr;

/** @brief Used to get the measurements from the blmc api. */
typedef blmc_drivers::MotorInterface::MeasurementIndex MeasurementIndex;

/** @brief Slider_ptr shortcut for the linear potentiometer analog sensor. */
typedef std::shared_ptr<blmc_drivers::AnalogSensor> Slider_ptr;

/**
 * @brief The class TeststandTi is used to control the TeststandTi robot located
 * at MPI-IS Tuebingen. The robot is composed of a single leg on a vertical
 * rail.
 */
class TeststandTi : public TeststandAbstractInterface
{
public:
    /**
     * @brief TeststandTi is the constructor of the class.
     */
    TeststandTi();

    /**
     * @copydoc TeststandAbstractInterface::initialize
     */
    virtual void initialize();

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

private:
    /**
     * @brief ATI sensor.
     */
    ati_ft_sensor::AtiFTSensor ati_sensor_;

    /**
     * @brief can_buses_ are the 2 can buses on the robot.
     */
    std::array<CanBus_ptr, 2> can_buses_;

    /**
     * @brief can_motor_boards_ are the 2 can motor board.
     */
    std::array<CanBusMotorBoard_ptr, 2> can_motor_boards_;

    /**
     * @brief motors_ are the objects allowing us to send motor commands and
     * receive data
     */
    std::array<MotorInterface_ptr, 2> motors_;

    /**
     * @brief joint_ptrs_ are the objects allowing us to send commands and
     * receive data at the joint level. It also ones some self calibration
     * routines.
     */
    blmc_drivers::BlmcJointModules<2> joints_;

    /**
     * @brief sliders_ these are analogue input from linear potentiometers.
     */
    std::array<Slider_ptr, 2> sliders_;

    /**
     * @brief contact_sensors_ is the contact sensors at each foot tips. They
     * also are analogue inputs.
     */
    std::array<ContactSensor_ptr, 1> contact_sensors_;

    /**
     * @brief contact_sensors_ is the contact sensors at each foot tips. They
     * also are analogue inputs.
     */
    std::array<HeightSensor_ptr, 1> height_sensors_;
};

}  // namespace teststand

#endif  // TeststandTi_H
