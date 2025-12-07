# Any C++ source files needed to build this target (drone_orchestrator_tester).
add_executable(drone_orchestrator_tester
  # list of source cpp files:
  main.cpp
  orchestrator_test.cpp
  )

# Any include directories needed to build this target.
# Note: we do not need to specify the include directories for the
# dependent libraries, they are automatically included.
target_include_directories(drone_orchestrator_tester PUBLIC
  # list of include directories:
  ${CMAKE_SOURCE_DIR}/include
  )

# Any dependent libraires needed to build this target.
# Note: RVO3D must be linked directly since drone_orchestrator is a static library
target_link_libraries(drone_orchestrator_tester PUBLIC
  # list of libraries:
  GTest::gtest
  # GTest::gmock
  RVO3D::RVO3D
  drone_orchestrator
  )

# Enable CMake’s test runner to discover the tests included in the
# binary, using the GoogleTest CMake module.
gtest_discover_tests(drone_orchestrator_tester)


# # Build executable:
# add_executable (drone_orchestrator_tester
#   test/orchestrator_test.cpp
#   )
# target_link_libraries(drone_orchestrator_tester
#   # list of libraries
#   drone_orchestrator
#   gtest
#   )
