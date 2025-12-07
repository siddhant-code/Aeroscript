# Test Documentation

This directory contains unit tests and integration tests for the drone_controller package.

## Test Files

### Unit Tests
- **helper_functions_test.cpp**: Tests for CSV parsing and utility functions
  - Tests `GetGoalsForLetter()` function
  - Tests file I/O and error handling
  - Tests case-insensitive letter matching

### Integration Tests
- **integration_test.cpp**: Original integration test for publisher node
- **subscriber_integration_test.cpp**: Integration tests for subscriber node
  - Tests position subscription and velocity publishing
  - Tests multiple drone coordination
  - Tests velocity command frequency

## Running Tests

### Build Tests
```bash
cd /path/to/workspace
colcon build --packages-select drone_controller
```

### Run All Tests
```bash
colcon test --packages-select drone_controller --event-handlers console_direct+
```

### Run Specific Test
```bash
# Run unit tests only
colcon test --packages-select drone_controller --event-handlers console_direct+ --pytest-args -k helper_functions

# Run integration tests only
colcon test --packages-select drone_controller --event-handlers console_direct+ --pytest-args -k integration
```

### View Test Results
```bash
colcon test-result --verbose
```

## Test Requirements

- ROS2 Humble
- catch_ros2 package (for integration tests)
- GoogleTest (for unit tests)
- drone_orchestrator package (dependency)

## Test Coverage

The tests cover:
- CSV file parsing for letter goals
- ROS2 topic communication
- Multi-drone coordination
- Velocity command publishing
- Message frequency validation

