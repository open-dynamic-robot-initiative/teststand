/**
 * @file dgm_teststandx.hpp
 * @author Manuel Wuthrich
 * @author Maximilien Naveau
 * @author Julian Viereck
 * @author Johannes Pfleging
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellshaft.
 */

#ifndef DGM_TESTSTAND_HH
#define DGM_TESTSTAND_HH

#include "teststand/teststand_ti.hpp"
#include "mim_msgs/srv/joint_calibration.hpp"
#include "dynamic_graph_manager/dynamic_graph_manager.hpp"
#include "yaml_utils/yaml_cpp_fwd.hpp"

namespace teststand
{
class DGMTeststandTi : public dynamic_graph_manager::DynamicGraphManager
{
public:
    /**
     * @brief DGMTeststandTi is the constructor.
     */
    DGMTeststandTi();

    /**
     * @brief ~DGMTeststandTi is the destructor.
     */
    ~DGMTeststandTi();

    /**
     * @brief initialize_hardware_communication_process is the function that
     * initialize the hardware.
     */
    void initialize_hardware_communication_process();

    /**
     * @brief get_sensors_to_map acquires the sensors data and feeds it to the
     * input/output map
     * @param[in][out] map is the sensors data filled by this function.
     */
    void get_sensors_to_map(dynamic_graph_manager::VectorDGMap& map);

    /**
     * @brief set_motor_controls_from_map reads the input map that contains the
     * controls and send these controls to the hardware.
     * @param map
     */
    void set_motor_controls_from_map(
        const dynamic_graph_manager::VectorDGMap& map);

    /**
     * @brief is_in_safety_mode Implement custom safe-mode detection.
     */
    virtual bool is_in_safety_mode();

    /**
     * @brief ROS callback
     *
     * @param req
     * @param res
     * @return true
     * @return false
     */
    void calibrate_joint_position_callback(
        mim_msgs::srv::JointCalibration::Request::SharedPtr req,
        mim_msgs::srv::JointCalibration::Response::SharedPtr res);

private:
    /**
     * @brief Calibrate the robot joint position
     *
     * @param zero_to_index_angle is the angle between the theoretical zero and
     * the next positive angle.
     */
    void calibrate_joint_position(const Eigen::Vector2d& zero_to_index_angle);

    /**
     * Entries for the real hardware.
     */

    /**
     * @brief test_bench_ the real test bench hardware drivers.
     */
    TeststandTi teststand_;

    /**
     * @brief ctrl_joint_torques_ the joint torques to be sent
     */
    Eigen::Vector2d ctrl_joint_torques_;

    /**
     * @brief These are the calibration value extracted from the paramters.
     * They represent the distance between the theorical zero joint angle and
     * the next joint index.
     */
    Eigen::Vector2d zero_to_index_angle_from_file_;

    /**
     * @brief was_in_safety_mode_ Toggle to keep in safety mode once it was
     * entered.
     */
    bool was_in_safety_mode_;
};

}  // namespace teststand

#endif  // DGM_TEST_BENCH_8_MOTORS_HH
