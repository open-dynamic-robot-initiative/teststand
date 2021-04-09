/**
 * \file teststand.cpp
 * \brief Execute the main program to control the teststand
 * \author Maximilien Naveau
 * \date 2018
 *
 * DynamicGraphManager for teststand main executbale.
 */

#include <fstream>
#include "teststand/dgm_teststand.hpp"

int main(int, char*[])
{
    // Get the dynamic_graph_manager config file.
    std::string yaml_path = DYNAMIC_GRAPH_MANAGER_YAML_PATH;

    std::cout << "Loading parameters from " << yaml_path << std::endl;
    
    std::ifstream f(yaml_path.c_str());
    if (!f.good())
    {
        throw std::runtime_error("Error: " + yaml_path + " not found!");
    }
    YAML::Node param = YAML::LoadFile(yaml_path);
    // Create the dgm.
    teststand::DGMTeststand dgm;

    // Initialize and run it.
    dgm.initialize(param);
    dgm.run();
}