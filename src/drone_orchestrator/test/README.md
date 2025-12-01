# Test Documentation

This directory contains unit tests for the drone_orchestrator package.

## Test Files

- **test.cpp**: Original dummy tests
- **orchestrator_test.cpp**: Comprehensive unit tests for Orchestrator class
  - Constructor initialization
  - Position and velocity setting
  - Goal setting and checking
  - Preferred velocity calculation
  - Goal achievement detection
  - Collision avoidance behavior
  - Edge cases

## Running Tests

### Build Tests
```bash
cd src/drone_orchestrator
cmake -S ./ -B build/
cmake --build build/
```

### Run Tests
```bash
cd build/
ctest --verbose
# or
./test/drone_orchestrator_tester
```

### Run Tests with Coverage
```bash
cmake -D COVERAGE=ON -S ./ -B build/
cmake --build build/ --clean-first
cmake --build build/ --target test_coverage
# View coverage report
open build/test_coverage/index.html
```

## Test Requirements

- CMake 3.14+
- GoogleTest
- RVO3D library
- C++17 compiler

## Test Coverage Goals

The tests aim for 80%+ code coverage of the Orchestrator class, including:
- All public methods
- Edge cases (empty vectors, single drone, etc.)
- Collision avoidance scenarios
- Goal achievement logic

