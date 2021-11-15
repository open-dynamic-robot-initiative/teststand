/**
 * \file dgm_teststand.cpp
 * \brief The hardware wrapper of the teststand robot
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file defines the TestBench8Motors class.
 */

#include "teststand/dgm_teststand.hpp"

#include "dynamic_graph_manager/ros.hpp"

namespace teststand
{
DGMTeststand::DGMTeststand()
{
    was_in_safety_mode_ = false;
}

DGMTeststand::~DGMTeststand()
{
}

void DGMTeststand::initialize_hardware_communication_process()
{
    /**
     * Load the calibration parameters
     */

    // get the hardware communication ros node handle
    dynamic_graph_manager::RosNodePtr ros_node_handle =
        dynamic_graph_manager::get_ros_node(
            dynamic_graph_manager::HWC_ROS_NODE_NAME);

    /** initialize the user commands */
    ros_user_commands_.push_back(
        ros_node_handle->create_service<mim_msgs::srv::JointCalibration>(
            "calibrate_joint_position",
            std::bind(&DGMTeststand::calibrate_joint_position_callback,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2)));

    std::string network_id;
    YAML::ReadParameter(
        params_["hardware_communication"], "network_id", network_id);

    teststand_.initialize(network_id);
}

void DGMTeststand::get_sensors_to_map(dynamic_graph_manager::VectorDGMap& map)
{
    teststand_.acquire_sensors();

    /**
     * Joint data
     */
    map.at("joint_positions") = teststand_.get_joint_positions();
    map.at("joint_velocities") = teststand_.get_joint_velocities();
    map.at("joint_torques") = teststand_.get_joint_torques();
    map.at("joint_target_torques") = teststand_.get_joint_target_torques();

    /**
     * Additional data
     */
    map.at("height_sensors") = teststand_.get_height_sensor();
    map.at("slider_positions") = teststand_.get_slider_positions();
    
    /**
     * Robot status
     */
    dynamicgraph::Vector& map_motor_enabled = map.at("motor_enabled");
    dynamicgraph::Vector& map_motor_ready = map.at("motor_ready");
    dynamicgraph::Vector& map_motor_board_enabled =
        map.at("motor_board_enabled");
    dynamicgraph::Vector& map_motor_board_errors = map.at("motor_board_errors");
    Eigen::Ref<const Eigen::Matrix<bool, TESTSTAND_NB_MOTOR, 1> > motor_enabled =
        teststand_.get_motor_enabled();
    Eigen::Ref<const Eigen::Matrix<bool, TESTSTAND_NB_MOTOR, 1> > motor_ready =
        teststand_.get_motor_ready();
    Eigen::Ref<const Eigen::Matrix<bool, TESTSTAND_NB_MOTOR_BOARD, 1> >
        motor_board_enabled = teststand_.get_motor_board_enabled();
    Eigen::Ref<const Eigen::Matrix<int, TESTSTAND_NB_MOTOR_BOARD, 1> >
        motor_board_errors = teststand_.get_motor_board_errors();

    for (unsigned i = 0; i < TESTSTAND_NB_MOTOR; ++i)
    {
        map_motor_enabled[i] = motor_enabled[i];
        map_motor_ready[i] = motor_ready[i];
    }
    for (unsigned i = 0; i < TESTSTAND_NB_MOTOR_BOARD; ++i)
    {
        map_motor_board_enabled[i] = motor_board_enabled[i];
        map_motor_board_errors[i] = motor_board_errors[i];
    }
}

void DGMTeststand::set_motor_controls_from_map(
    const dynamic_graph_manager::VectorDGMap& map)
{
    try
    {
        // here we need to perform and internal copy. Otherwise the compilator
        // complains
        ctrl_joint_torques_ = map.at("ctrl_joint_torques");
        // Actually send the control to the robot
        teststand_.send_target_joint_torque(ctrl_joint_torques_);
    }
    catch (const std::exception& e)
    {
        rt_printf(
            "DGMTeststand::set_motor_controls_from_map: "
            "Error sending controls, %s\n",
            e.what());
    }
}

void DGMTeststand::calibrate_joint_position_callback(
    mim_msgs::srv::JointCalibration::Request::SharedPtr,
    mim_msgs::srv::JointCalibration::Response::SharedPtr res)
{
    // parse and register the command for further call.
    add_user_command(std::bind(&DGMTeststand::calibrate_joint_position, this));

    // return whatever the user want
    res->sanity_check = true;
}

void DGMTeststand::calibrate_joint_position()
{
    teststand_.request_calibration();
}

}  // namespace teststand
