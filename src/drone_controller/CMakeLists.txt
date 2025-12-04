cmake_minimum_required(VERSION 3.10)
project(drone_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###################################
# find dependencies
###################################
find_package(ament_cmake REQUIRED) # will enable the BUILD_TESTING option
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Doxygen REQUIRED)    # add doxygen support
find_package(drone_orchestrator REQUIRED)   # add drone_orchestrator
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
find_package(RVO3D REQUIRED)

###################################################
# Build level 2 unit test (i.e, integration test)
###################################################
# below sets up code coverage flags
if(COVERAGE)
  set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_CXX_FLAGS "--coverage")
  set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE "ON")
endif()

if(BUILD_TESTING)
  find_package(catch_ros2 REQUIRED) # add integration test support
  find_package(ament_cmake_gtest REQUIRED) # add unit test support
  
  # Original integration test
  add_executable(integration_test
    test/integration_test.cpp)
  ament_target_dependencies(integration_test
    rclcpp
    std_msgs
    geometry_msgs 
    RVO3D)
  target_link_libraries(integration_test
    catch_ros2::catch_ros2_with_node_main
    drone_orchestrator)
  catch_ros2_add_integration_test (Demo_Integration_Test
    LAUNCH_FILE integration_test.launch.yaml)
  
  # New subscriber integration test
  add_executable(subscriber_integration_test
    test/subscriber_integration_test.cpp)
  
  ament_target_dependencies(subscriber_integration_test
    rclcpp
    std_msgs
    geometry_msgs
    RVO3D)
  
  target_link_libraries(subscriber_integration_test
    catch_ros2::catch_ros2_with_node_main
    drone_orchestrator)
  catch_ros2_add_integration_test (Subscriber_Integration_Test
    LAUNCH_FILE subscriber_integration_test.launch.yaml
    TIMEOUT 30)
  
  # Unit tests for helper functions
  ament_add_gtest(helper_functions_test
    test/helper_functions_test.cpp
  )
  if(TARGET helper_functions_test)
    target_include_directories(helper_functions_test PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    ament_target_dependencies(helper_functions_test
      rclcpp
      geometry_msgs
      RVO3D)
    target_link_libraries(helper_functions_test
      drone_orchestrator)
  endif()
endif()

###################################################
# Create the compilation database for clangd and move it out of the build dir.
###################################################
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Enable compile_commands.json
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")

###################################################
# Doxygen Rules
# ref: https://cmake.org/cmake/help/latest/module/FindDoxygen.html
###################################################
if (DOXYGEN_FOUND)
  set( DOXYGEN_OUTPUT_DIRECTORY     ${PROJECT_SOURCE_DIR}/docs )
  set( DOXYGEN_COLLABORATION_GRAPH  YES )
  set( DOXYGEN_EXTRACT_ALL          YES )
  set( DOXYGEN_CLASS_DIAGRAMS       YES )
  set( DOXYGEN_HIDE_UNDOC_RELATIONS NO )
  set( DOXYGEN_HAVE_DOT             YES )
  set( DOXYGEN_CLASS_GRAPH          YES )
  set( DOXYGEN_CALL_GRAPH           YES )
  set( DOXYGEN_CALLER_GRAPH         YES )
  set( DOXYGEN_COLLABORATION_GRAPH  YES )
  set( DOXYGEN_BUILTIN_STL_SUPPORT  YES )
  set( DOXYGEN_EXTRACT_PRIVATE      YES )
  set( DOXYGEN_EXTRACT_PACKAGE      YES )
  set( DOXYGEN_EXTRACT_STATIC       YES )
  set( DOXYGEN_EXTRACT_LOCALMETHODS YES )
  set( DOXYGEN_UML_LOOK             YES )
  set( DOXYGEN_UML_LIMIT_NUM_FIELDS 50 )
  set( DOXYGEN_TEMPLATE_RELATIONS   YES )
  set( DOXYGEN_DOT_GRAPH_MAX_NODES  100 )
  set( DOXYGEN_MAX_DOT_GRAPH_DEPTH  0 )
  set( DOXYGEN_DOT_TRANSPARENT      YES )
else()
  message( FATAL_ERROR "Doxygen needs to be installed to generate the doxygen documentation" )
endif()

###################################
# Specify Targets
###################################
# create a target to build documentation
doxygen_add_docs(docs           # target name
  # List of files or directories
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/test)

# Add executables:
add_executable(talker
  src/publisher_member_function.cpp)
add_executable(listener
  src/subscriber_member_function.cpp)

# Add executable dependencis:
ament_target_dependencies(talker
  rclcpp std_msgs geometry_msgs)
target_link_libraries(talker
  drone_orchestrator)

ament_target_dependencies(listener
  rclcpp std_msgs geometry_msgs RVO3D)
target_link_libraries(listener
  drone_orchestrator)

###################################
# Installation
###################################
# Install executables:
install(TARGETS
  talker
  listener
  integration_test
  subscriber_integration_test
  helper_functions_test
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)


# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

## Install some arbitrary executables
install(PROGRAMS      # want to make the file executable (ie. chmod a+x)
  scripts/run_me_for_fun.bash
  scripts/generate_coverage_report.bash
  DESTINATION lib/${PROJECT_NAME})

ament_package()
