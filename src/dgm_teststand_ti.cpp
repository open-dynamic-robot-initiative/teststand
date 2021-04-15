/**
 * \file dg_teststand.cpp
 * \brief DGM wrapper around the teststand robot.
 * \author Julian Viereck
 * \date 2018
 */

#include "teststand/dgm_teststand_ti.hpp"
#include "dynamic_graph_manager/ros.hpp"

namespace teststand
{
DGMTeststandTi::DGMTeststandTi() : was_in_safety_mode_(false)
{
}

DGMTeststandTi::~DGMTeststandTi()
{
}

void DGMTeststandTi::initialize_hardware_communication_process()
{
    /**
     * Load the calibration parameters
     */
    Eigen::Vector2d joint_index_to_zero;
    YAML::ReadParameter(params_["hardware_communication"]["calibration"],
                        "index_to_zero_angle",
                        zero_to_index_angle_from_file_);

    // Get the hardware communication ros node handle.
    dynamic_graph_manager::RosNodePtr ros_node_handle =
        dynamic_graph_manager::get_ros_node(
            dynamic_graph_manager::HWC_ROS_NODE_NAME);

    /** Initialize the user commands. */
    ros_user_commands_.push_back(
        ros_node_handle->create_service<mim_msgs::srv::JointCalibration>(
            "calibrate_joint_position",
            std::bind(&DGMTeststandTi::calibrate_joint_position_callback,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2)));

    /**
     * Initialize the hardware
     */
    teststand_.initialize();
}

bool DGMTeststandTi::is_in_safety_mode()
{
    was_in_safety_mode_ |=
        teststand_.get_joint_velocities().cwiseAbs().maxCoeff() > 10000.;
    if (was_in_safety_mode_ || DynamicGraphManager::is_in_safety_mode())
    {
        was_in_safety_mode_ = true;
        return true;
    }
    else
    {
        return false;
    }
}

void DGMTeststandTi::get_sensors_to_map(dynamic_graph_manager::VectorDGMap& map)
{
    try
    {
        teststand_.acquire_sensors();
        /**
         * Joint data
         */
        map.at("joint_positions") = teststand_.get_joint_positions();
        map.at("joint_velocities") = teststand_.get_joint_velocities();
        map.at("joint_torques") = teststand_.get_joint_torques();
        map.at("joint_target_torques") = teststand_.get_joint_target_torques();
        map.at("joint_encoder_index") = teststand_.get_joint_encoder_index();

        /**
         * Additional data
         */
        map.at("height_sensors") = teststand_.get_height_sensor();
        map.at("ati_force") = teststand_.get_ati_force();
        map.at("ati_torque") = teststand_.get_ati_torque();
    }
    catch (...)
    {
        printf("Error in acquiring the sensors data\n");
        printf("Setting all of them 0.0\n");

        /**
         * Joint data
         */
        map.at("joint_positions").fill(0.0);
        map.at("joint_velocities").fill(0.0);
        map.at("joint_torques").fill(0.0);
        map.at("joint_target_torques").fill(0.0);
        map.at("joint_encoder_index").fill(0.0);

        /**
         * Additional data
         */
        map.at("contact_sensors").fill(0.0);
        map.at("slider_positions").fill(0.0);
        map.at("height_sensors").fill(0.0);

        map.at("ati_force").fill(0.0);
        map.at("ati_torque").fill(0.0);
    }
}

void DGMTeststandTi::set_motor_controls_from_map(
    const dynamic_graph_manager::VectorDGMap& map)
{
    try
    {
        ctrl_joint_torques_ = map.at("ctrl_joint_torques");
        teststand_.send_target_joint_torque(ctrl_joint_torques_);
    }
    catch (...)
    {
        printf("Error sending controls\n");
    }
}

void DGMTeststandTi::calibrate_joint_position_callback(
    mim_msgs::srv::JointCalibration::Request::SharedPtr,
    mim_msgs::srv::JointCalibration::Response::SharedPtr res)
{
    // parse and register the command for further call.
    add_user_command(std::bind(&DGMTeststandTi::calibrate_joint_position,
                               this,
                               zero_to_index_angle_from_file_));

    // return whatever the user want
    res->sanity_check = true;
}

void DGMTeststandTi::calibrate_joint_position(
    const Eigen::Vector2d& zero_to_index_angle)
{
    teststand_.calibrate(zero_to_index_angle);
}

}  // namespace teststand
