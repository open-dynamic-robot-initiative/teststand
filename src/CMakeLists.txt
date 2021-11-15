set(all_lib_targets)
set(all_exe_targets)

#
# YAML parameters.
#
string(
  CONCAT odri_control_interface_yaml_path
         "${PythonModules_robot_properties_teststand_PATH}/"
         "robot_properties_teststand/robot_properties_teststand/"
         "odri_control_interface/teststand_driver.yaml")
string(
  CONCAT dynamic_graph_manager_yaml_file
         "${PythonModules_robot_properties_teststand_PATH}/"
         "robot_properties_teststand/robot_properties_teststand/"
         "dynamic_graph_manager/dgm_parameters_teststand.yaml")

#
# Declare the teststand library.
#
add_library(teststand SHARED teststand.cpp)
# Add includes
target_include_directories(
  teststand PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                   $<INSTALL_INTERFACE:include>)
# Link the dependencies.
target_link_libraries(teststand slider_box::slider_box)
target_link_libraries(teststand odri_control_interface::odri_control_interface)
target_link_libraries(teststand real_time_tools::real_time_tools)
target_link_libraries(teststand yaml_utils::yaml_utils)
target_link_libraries(teststand Eigen3::Eigen)
target_compile_definitions(
  teststand
  PUBLIC ODRI_CONTROL_INTERFACE_YAML_PATH="${odri_control_interface_yaml_path}")
# Export the target.
list(APPEND all_lib_targets teststand)

#
# Teststand hardware calibration program.
#
add_executable(teststand_hardware_calibration
               programs/hardware_calibration_teststand.cpp)
# Add the include dependencies.
target_include_directories(
  teststand_hardware_calibration
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
# Link the dependencies.
target_link_libraries(teststand_hardware_calibration teststand)
# Export the target.
list(APPEND all_exe_targets teststand_hardware_calibration)


#
# Declare the teststand_ti library.
#
if (BUILD_TESTSTAND_TI)
  add_library(teststand_ti SHARED teststand_ti.cpp)
  # Add includes
  target_include_directories(
    teststand_ti PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                        $<INSTALL_INTERFACE:include>)
  # Link the dependencies.
  target_link_libraries(teststand_ti blmc_driver::blmc_driver)
  target_link_libraries(teststand_ti real_time_tools::real_time_tools)
  target_link_libraries(teststand_ti yaml_utils::yaml_utils)
  target_link_libraries(teststand_ti Eigen3::Eigen)
  # Export the target.
  list(APPEND all_lib_targets teststand_ti)

  #
  # Teststand hardware calibration program.
  #
  add_executable(teststand_ti_hardware_calibration
                programs/hardware_calibration_teststand_ti.cpp)
  # Add the include dependencies.
  target_include_directories(
    teststand_ti_hardware_calibration
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>)
  # Link the dependencies.
  target_link_libraries(teststand_ti_hardware_calibration teststand_ti)
  # Export the target.
  list(APPEND all_exe_targets teststand_ti_hardware_calibration)
endif()

#
# Optionally compile the dynamic_graph_manager Teststand wrapper.
#
if(${dynamic_graph_manager_FOUND})
  #
  # create dgm_teststand library
  #
  add_library(dgm_teststand SHARED dgm_teststand.cpp)
  # Add the include dependencies.
  target_include_directories(
    dgm_teststand PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)
  # Link the dependencies.
  target_link_libraries(dgm_teststand ${PROJECT_NAME})
  target_link_libraries(dgm_teststand
                        dynamic_graph_manager::dynamic_graph_manager)
  # Export the target.
  list(APPEND all_lib_targets dgm_teststand)

  #
  # Create the teststand executable.
  #
  add_executable(dg_main_teststand programs/dg_main_teststand.cpp)
  # Add the include dependencies.
  target_include_directories(
    dg_main_teststand
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
           ${dynamic_graph_manager_INCLUDE_DIR} $<INSTALL_INTERFACE:include>)
  # Link the dependencies.
  target_link_libraries(dg_main_teststand dgm_teststand)
  target_compile_definitions(
    dg_main_teststand
    PUBLIC
      DYNAMIC_GRAPH_MANAGER_YAML_PATH="${dynamic_graph_manager_yaml_file}"
      ODRI_CONTROL_INTERFACE_YAML_PATH="${odri_control_interface_yaml_path}")
  # Export.
  list(APPEND all_exe_targets dg_main_teststand)

  if (BUILD_TESTSTAND_TI)
    #
    # create dgm_teststand library
    #
    add_library(dgm_teststand_ti SHARED dgm_teststand_ti.cpp)
    # Add the include dependencies.
    target_include_directories(
      dgm_teststand_ti PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                              $<INSTALL_INTERFACE:include>)
    # Link the dependencies.
    target_link_libraries(dgm_teststand_ti teststand_ti)
    target_link_libraries(dgm_teststand_ti
                          dynamic_graph_manager::dynamic_graph_manager)
    # Export the target.
    list(APPEND all_lib_targets dgm_teststand_ti)

    #
    # Create the teststand executable.
    #
    add_executable(dg_main_teststand_ti programs/dg_main_teststand_ti.cpp)
    # Add the include dependencies.
    target_include_directories(
      dg_main_teststand_ti
      PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
            ${dynamic_graph_manager_INCLUDE_DIR} $<INSTALL_INTERFACE:include>)
    # Link the dependencies.
    target_link_libraries(dg_main_teststand_ti dgm_teststand_ti)
    target_compile_definitions(
      dg_main_teststand_ti
      PUBLIC
        DYNAMIC_GRAPH_MANAGER_YAML_PATH="${dynamic_graph_manager_yaml_file}"
        ODRI_CONTROL_INTERFACE_YAML_PATH="${odri_control_interface_yaml_path}")
    # Export.
    list(APPEND all_exe_targets dg_main_teststand_ti)
  endif()
endif(${dynamic_graph_manager_FOUND})

#
# Install and Export the libraries an programs.
#
install(
  TARGETS ${all_lib_targets} ${all_exe_targets}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)

foreach(target ${all_exe_targets})
  # install a symlink of the executable in lib/teststand
  string(
    CONCAT symlink_command
           "execute_process("
           "  COMMAND ${CMAKE_COMMAND} -E make_directory "
           "  ${CMAKE_INSTALL_PREFIX}/bin/)\n"
           "execute_process("
           "  COMMAND ${CMAKE_COMMAND} -E create_symlink "
           "  ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/${target}"
           "  ${CMAKE_INSTALL_PREFIX}/bin/${target} )")
  install(CODE ${symlink_command})
endforeach()
