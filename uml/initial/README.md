# UML Diagrams for Drone System Architecture

This directory contains UML diagrams documenting the software design of the multi-drone system.

## Diagrams

### 1. Class Diagram (`class_diagram.puml`)
Shows the class structure and relationships in the system:
- **Orchestrator**: Core class using RVO (Reciprocal Velocity Obstacles) for multi-agent path planning
- **MinimalSubscriber**: ROS2 node that coordinates multiple drones
- **MavicDriver**: Python class for Webots drone control
- **ROS2 Messages**: Message types used for communication
- Relationships: inheritance, composition, and dependencies

### 2. Dependency Graph (`dependency_graph.puml`)
Illustrates package and file dependencies:
- External dependencies (RVO3D, ROS2, OpenCV, Webots, etc.)
- Internal file dependencies within packages
- Package-level dependencies between `drone_orchestrator`, `drone_controller`, and `mavic_simulation`

### 3. Sequence Diagram (`sequence_diagram.puml`)
Shows the message flow and interaction between components:
- System initialization sequence
- Main control loop (8ms timer)
- Communication between Webots, MavicDriver, ROS2 topics, Controller, and Orchestrator
- Goal achievement and letter sequence handling

### 4. Activity Diagram (`activity_diagram.puml`)
Describes the control flow and decision logic:
- System initialization steps
- Main control loop with RVO path planning
- Goal checking and letter queue management
- Sensor reading and actuator control flow

### 5. Component Diagram (`component_diagram.puml`)
Shows system components and their interfaces:
- Build system components (CMake, colcon)
- Software packages and their relationships
- ROS2 infrastructure (topics, DDS)
- Simulation components (Webots, MavicDriver)

## Generating Diagrams

### Prerequisites
- Java (for PlantUML JAR)
- PlantUML JAR file (included in this directory)

### Generate PNG images
```bash
cd uml
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
3. **mavic_simulation**: ROS2 Python package providing Webots simulation environment with MavicDriver for individual drone control

The system enables multiple drones to collaboratively write letters in the air by:
- Reading GPS positions and velocities from simulated drones
- Using RVO algorithm for collision-free path planning
- Publishing velocity commands to control drone movement
- Managing a sequence of letters to write

