#
# Copyright (c) 2019, New York University and Max Planck Gesellschaft. License
# BSD-3 clause
#

#
# set up the project
#
cmake_minimum_required(VERSION 3.10.2)

project(teststand VERSION 1.0.0)

# specify the C++ 17 standard.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# external dependencies.
find_package(mpi_cmake_modules REQUIRED)
find_package(pybind11 REQUIRED)

# local dependencies.
find_package(ament_cmake REQUIRED)
find_package(slider_box REQUIRED)
find_package(odri_control_interface REQUIRED)
find_package(real_time_tools REQUIRED)
find_package(yaml_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(PythonModules REQUIRED COMPONENTS robot_properties_teststand)

# optional dependencies.
find_package(blmc_driver QUIET)
find_package(dynamic_graph_manager QUIET)

# Build and export depending on weather blmc_drivers was found or not.
set(BUILD_TESTSTAND_TI off)
set(export_list
  odri_control_interface real_time_tools yaml_utils dynamic_graph_manager
  rclcpp rclcpp_components
)
if(${blmc_drivers_FOUND})
  set(BUILD_TESTSTAND_TI on)
  set(export_list ${export_list} blmc_driver)
endif()
ament_export_dependencies(${export_list})

# prepare the final export
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#
# manage the creation of the libraries and executables
#
add_subdirectory(src)

#
# Manage the unit tests.
#
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

#
# Manage the demos.
#
add_subdirectory(demos)

#
# Python bindings.
#
add_subdirectory(srcpy)

#
# Install the package
#

# Install includes.
install(DIRECTORY include/ DESTINATION include)

# Install python files.
get_python_install_dir(python_install_dir)
install(
  DIRECTORY python/${PROJECT_NAME}
  DESTINATION "${python_install_dir}"
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE)

#
# Building documentation.
#
add_documentation()

#
# Export this package as a cmake package.
#
ament_package()
