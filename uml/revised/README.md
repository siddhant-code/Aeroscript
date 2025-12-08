# Revised UML Diagrams for Drone System Architecture

This directory contains updated UML diagrams reflecting the current codebase structure after recent changes.

## Changes from Initial Diagrams

### Key Updates:
1. **Class Renaming**: `MinimalSubscriber` → `DroneManager`
2. **New Executable**: `mavic_controller` (main controller node)
3. **Helper Function**: `GetGoalsForLetter()` is now a standalone function
4. **Launch Parameters**: Added `text` parameter (default: "HEY") and `csv_file` parameter
5. **Test Infrastructure**: Added unit tests (`helper_functions_test`) and integration tests (`subscriber_integration_test`)
6. **Test Utilities**: New `test_utils.hpp` namespace with helper functions
7. **Parameter Handling**: Uses `ament_index_cpp` for package resource resolution
8. **Dynamic World Generation**: Launch file generates world files with configurable number of drones (default: 20)

## Diagrams

### 1. Class Diagram (`class_diagram.puml`)
Shows the updated class structure:
- **DroneManager**: Main controller class (renamed from MinimalSubscriber)
- **GetGoalsForLetter**: Standalone function for CSV parsing
- **Orchestrator**: Core RVO path planning class
- **test_utils**: Namespace with test helper functions
- **HelperFunctionsTest**: Unit test class for CSV parsing

### 2. Component Diagram (`component_diagram.puml`)
Illustrates system components:
- **mavic_controller**: Main executable (DroneManager node)
- **robot_launch.py**: Launch file with parameter support
- **World Generator**: Dynamic world file generation
- **Parameters**: ROS2 parameters (text, csv_file)
- **CSV Config**: Letter goals configuration file
- Updated test components (unit and integration tests)

### 3. Sequence Diagram (`sequence_diagram.puml`)
Shows the message flow:
- Launch file parameter declaration and setting
- CSV file loading via `GetGoalsForLetter()`
- Parameter reading in DroneManager
- Updated control loop with parameter-based CSV path resolution

### 4. Activity Diagram (`activity_diagram.puml`)
Describes the control flow:
- Launch argument declaration
- Dynamic world generation
- Parameter reading (text, csv_file)
- CSV file loading for letter goals
- Updated letter queue management

### 5. Dependency Graph (`dependency_graph.puml`)
Shows dependencies:
- **ament_index_cpp**: For package resource resolution
- **Catch2/catch_ros2**: For integration tests
- **test_utils.hpp**: Test utility functions
- Updated file dependencies (dji_mavic_controller.cpp, helper_functions_test.cpp, etc.)
- CSV file as a dependency

## Generating Diagrams

### Prerequisites
- Java (for PlantUML JAR)
- PlantUML JAR file (copy from `../initial/plantuml.jar` or download)

### Generate PNG images
```bash
cd uml/revised
java -jar ../initial/plantuml.jar -tpng *.puml
```

### Generate SVG images
```bash
java -jar ../initial/plantuml.jar -tsvg *.puml
```

### Generate all formats
```bash
java -jar ../initial/plantuml.jar *.puml
```

## Architecture Updates

### Parameter System
- **text parameter**: Specifies the text string to write (default: "HEY")
- **csv_file parameter**: Path to CSV file with letter goals (default: package share directory)

### CSV File Handling
- CSV file is installed to `share/drone_controller/config/`
- Path resolution uses `ament_index_cpp::get_package_share_directory()`
- `GetGoalsForLetter()` function parses CSV and extracts goals for specific letters

### Testing Infrastructure
- **Unit Tests**: `helper_functions_test.cpp` tests CSV parsing
- **Integration Tests**: `subscriber_integration_test.cpp` tests ROS2 node communication
- **Test Utilities**: `test_utils.hpp` provides helper functions for testing

### Launch File Enhancements
- Dynamic world generation with configurable number of drones
- Parameter support for text and CSV file paths
- Automatic world file generation before simulation start

