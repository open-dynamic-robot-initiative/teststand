/**
 * \file demo_dg_test_bench_8_motors.cpp
 * \brief The use of the wrapper implementing a small pid controller.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the TestBench8Motors class in a small demo.
 */

#include "teststand/dgm_teststand_ti.hpp"

int main(int, char*[])
{
    // Get the dynamic_graph_manager config file.
    std::string yaml_path = DYNAMIC_GRAPH_MANAGER_YAML_PATH;
    std::cout << "Loading paramters from " << yaml_path << std::endl;
    YAML::Node param = YAML::LoadFile(yaml_path);

    // Create the dgm.
    teststand::DGMTeststandTi dgm;

    // Initialize and run it.
    dgm.initialize(param);
    dgm.run();

    // Wait until ROS is shutdown.
    std::cout << "Wait for shutdown, press CTRL+C to close." << std::endl;
    dynamic_graph_manager::ros_spin();
    dynamic_graph_manager::ros_shutdown();
}
