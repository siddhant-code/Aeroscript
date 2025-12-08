#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots Mavic 2 Pro driver."""

import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
import shutil
from launch_ros.actions import Node


def generate_launch_description():
    def generate_wbt_file(num_drones):
        with open(
            "install/mavic_simulation/share/mavic_simulation/worlds/mavic_world.wbt"
        ) as template_file:
            template_content = template_file.read()

        # Cria drones de acordo com a quantidade desejada
        wbt_content = ""
        y = 0
        for i in range(num_drones):
            # Modify template content for each drone instance
            drone_content = f"Mavic2Pro {{\n"
            drone_content += (
                f"  translation 0 {y} 0.07\n"  # Adjust translation based on 'i'
            )
            drone_content += "  rotation 0 0 1 3.141590777218456\n"
            drone_content += f'  name "Mavic_2_PRO_{i + 1}"\n'
            drone_content += '  controller "<extern>"\n'
            drone_content += "  supervisor TRUE\n"
            drone_content += "  cameraSlot [\n"
            drone_content += "    Camera {\n"
            drone_content += "      width 400\n"
            drone_content += "      height 240\n"
            drone_content += "      near 0.2\n"
            drone_content += "      rotation 0 1 0 1.5708\n"
            drone_content += "    }\n"
            drone_content += "  ]\n"
            drone_content += "}\n\n"

            wbt_content += drone_content
            y += 1

        wbt_content = template_content + wbt_content

        if os.path.exists(
            "install/mavic_simulation/share/mavic_simulation/worlds/updated_world.wbt"
        ):
            os.remove(
                "install/mavic_simulation/share/mavic_simulation/worlds/updated_world.wbt"
            )

        destination_dir = "install/mavic_simulation/share/mavic_simulation/worlds/"

        with open("updated_world.wbt", "w") as output_file:
            output_file.write(wbt_content)

        shutil.move("updated_world.wbt", destination_dir)

    package_dir = get_package_share_directory("mavic_simulation")
    world = LaunchConfiguration("world")

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, "worlds", world]), ros2_supervisor=True
    )
    robot_description_path = os.path.join(package_dir, "resource", "mavic_webots.urdf")

    # num_drones = int(input("\n****************************************"
    # +"\nHow many drones do you want to simulate?"
    # +"\n****************************************\nR:"))
    num_drones = 20

    generate_wbt_file(num_drones)

    mavic_drivers = {}
    for i in range(num_drones):
        driver_name = f"mavic_driver_{i + 1}"
        drone_name = f"Mavic_2_PRO_{i + 1}"

        temporary_path = os.path.join(
            package_dir, "resource", f"mavic_webots_temp_{i + 1}.urdf"
        )

        with open(robot_description_path, "r") as original_file:
            content = original_file.read()

        content = content.replace("/imu", f"/Mavic_2_PRO_{i + 1}/imu")

        with open(temporary_path, "w+") as temp_file:
            temp_file.write(content)

        mavic_drivers[driver_name] = WebotsController(
            robot_name=drone_name,
            parameters=[
                {"robot_description": temporary_path},
            ],
            respawn=True,
        )

    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="updated_world.wbt",
                description="Choose one of the world files from `/mavic_simulation/worlds` directory",
            ),
            DeclareLaunchArgument(
                "text",
                default_value="HEY",
                description="Text string to be written by the drones",
            ),
            webots,
            webots._supervisor,
            # This action will kill all nodes once the Webots simulation has exited
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )

    for mavic in mavic_drivers:
        ld.add_action(mavic_drivers[mavic])

    sleep = launch.actions.ExecuteProcess(
        cmd=["sleep", "15"],  # Sleep for 3 seconds before launching the second node
        name="sleep_to_start_sim",
    )
    ld.add_action(sleep)

    pkg_share_dir = get_package_share_directory("drone_controller")
    csv_path = pkg_share_dir + "/config/letters_AZ_cad.csv"
    drone_controller = Node(
        package="drone_controller",
        executable="mavic_controller",
        parameters=[
            {"text": LaunchConfiguration("text")},
            {"csv_file": LaunchConfiguration("csv_file", default=csv_path)},
        ],
    )
    ld.add_action(drone_controller)

    return ld
