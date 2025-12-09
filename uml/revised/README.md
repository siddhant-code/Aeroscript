# Revised UML Diagrams for AeroScript

This directory contains updated UML diagrams reflecting the current implementation of the AeroScript multi-drone coordination system.

## Diagrams

### 1. Class Diagram (`class_diagram.puml`)
Shows the class structure and relationships in the system:
- **Orchestrator**: Core class using RVO (Reciprocal Velocity Obstacles) for multi-agent path planning
- **DroneManager**: ROS2 node (renamed from MinimalSubscriber) that coordinates multiple drones
- **MavicDriver**: C++ Webots plugin for individual drone control
- **GetGoalsForLetter**: Standalone function for CSV parsing
- **ROS2 Messages**: Message types used for communication
- **RVO Classes**: RVO::RVOSimulator and RVO::Vector3 from RVO3D library

**Key Updates:**
- Class renamed from `MinimalSubscriber` to `DroneManager`
- Updated method names to match Google C++ Style Guide
- Added `GetGoalsForLetter` as standalone function
- Updated MavicDriver to C++ implementation
- Added ROS2 parameter handling

### 2. Component Diagram (`component_diagram.puml`)
Illustrates system components and their interfaces:
- **Build System**: CMake and colcon
- **Software Packages**: drone_orchestrator, drone_controller, mavic_simulation
- **ROS2 Infrastructure**: Topics, DDS, Parameters, Bag Recorder
- **Simulation Components**: Webots, MavicDriver, World Generator
- **Testing Components**: Unit tests, Integration tests

**Key Updates:**
- Added `mavic_controller` executable
- Added World Generator component
- Added ROS2 Parameters component
- Added ROS2 Bag Recorder component
- Added CSV Config component
- Updated MavicDriver to C++ plugin
- Updated test components

### 3. Sequence Diagram (`sequence_diagram.puml`)
Shows the message flow and interaction between components:
- System initialization sequence with launch arguments
- Main control loop 
- Communication between Webots, MavicDriver, ROS2 topics, Controller, and Orchestrator
- Goal achievement and letter sequence handling
- CSV file loading and parameter handling

**Key Updates:**
- Added launch argument declaration
- Added CSV file loading sequence
- Updated to show parameter reading in DroneManager
- Added bag recording optional flow
- Updated class names (DroneManager instead of MinimalSubscriber)
- Added detailed RVO algorithm notes

### 4. Activity Diagram (`activity_diagram.puml`)
Describes the control flow and decision logic:
- System initialization steps
- Launch argument handling
- CSV file path resolution
- Main control loop with RVO path planning
- Goal checking and letter queue management
- Sensor reading and actuator control flow
- Bag recording conditional flow

**Key Updates:**
- Added launch argument declaration flow
- Added dynamic world generation
- Added parameter reading and CSV path resolution
- Added bag recording conditional logic
- Updated control flow to match current implementation

### 5. Dependency Graph (`dependency_graph.puml`)
Shows package and file dependencies:
- External dependencies (RVO3D, ROS2, OpenCV, Webots, etc.)
- Internal file dependencies within packages
- Package-level dependencies
- Test dependencies (GoogleTest, Catch2/catch_ros2)
- Build system dependencies

**Key Updates:**
- Updated file names (dji_mavic_controller.cpp instead of subscriber_member_function.cpp)
- Added ament_index_cpp dependency
- Added Catch2/catch_ros2 for integration tests
- Updated MavicDriver to C++ files
- Added CSV file dependencies
- Added launch file dependencies

## Generating Diagrams

### Prerequisites
- Java (for PlantUML JAR)
- PlantUML JAR file (included in this directory)

### Generate PNG images
```bash
cd uml/revised
java -jar plantuml.jar -tpng *.puml
```

### Generate SVG images
```bash
java -jar plantuml.jar -tsvg *.puml
```

### Generate all formats
```bash
java -jar plantuml.jar *.puml
```

## Editing Diagrams

The `.puml` files are PlantUML source files. You can edit them with any text editor. PlantUML syntax documentation: https://plantuml.com/

## Viewing Diagrams

- **PNG files**: Can be viewed with any image viewer
- **PlantUML files**: Can be viewed with PlantUML plugins in:
  - VS Code (PlantUML extension)
  - IntelliJ IDEA
  - Online at http://www.plantuml.com/plantuml/uml/

## Architecture Overview

The system consists of three main packages:

1. **drone_orchestrator**: Stand-alone C++ library for multi-agent path planning using RVO algorithm
2. **drone_controller**: ROS2 package that coordinates multiple drones, subscribes to sensor data, and publishes velocity commands
3. **mavic_simulation**: ROS2 C++ package providing Webots simulation environment with MavicDriver for individual drone control

The system enables multiple drones to collaboratively write letters in the air by:
- Reading GPS positions and velocities from simulated drones
- Using RVO algorithm for collision-free path planning
- Publishing velocity commands to control drone movement
- Managing a sequence of letters to write
- Loading letter waypoints from CSV files
- Supporting launch arguments for text input and bag recording

## Key Features Documented

- **MVC Architecture**: Model (Orchestrator), View (MavicDriver/Webots), Controller (DroneManager)
- **RVO Algorithm**: Reciprocal Velocity Obstacles for collision avoidance
- **CSV Waypoint Loading**: Letter patterns stored in CSV format
- **ROS2 Parameters**: Configurable text input and CSV file paths
- **Bag Recording**: Optional ROS2 bag recording with topic filtering
- **Dynamic World Generation**: World file generated based on number of drones

