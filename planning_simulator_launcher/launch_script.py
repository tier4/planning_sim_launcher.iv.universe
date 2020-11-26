from pathlib import Path

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml import Parser

from launch import LaunchContext, LaunchDescription
from launch.actions import (ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler, Shutdown)
from launch.event import Event
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_description(*, launch_path, vehicle_model, scenario_runner_args, included_launch_file_args):
    vehicle_package = f'{vehicle_model}_description'
    vehicle_package_path = FindPackageShare(vehicle_package).find(vehicle_package)
    vehicle_info_param_path = Path(vehicle_package_path) / 'config' / 'vehicle_info.yaml'
    initial_engage_state = 'True'
    scenario_runner = Node(
        package='scenario_runner',
        executable='scenario_runner_node',
        parameters=[
            scenario_runner_args,
            vehicle_info_param_path,
        ],
        remappings=[
            ("input/pointcloud", "/sensing/lidar/no_ground/pointcloud"),
            ("input/vectormap", "/map/vector_map"),
            ("input/route", "/planning/mission_planning/route"),
            ("input/autoware_state", "/autoware/state"),
            ("input/vehicle_twist", "/vehicle/status/twist"),
            ("input/signal_command", "/vehicle/status/turn_signal"),
            ("output/start_point", "/initialpose"),
            ("output/initial_velocity", "/initialtwist"),
            ("output/goal_point", "/planning/mission_planning/goal"),
            ("output/check_point", "/planning/mission_planning/checkpoint"),
            ("output/autoware_engage", "/autoware/engage"),
            ("output/simulator_engage", "/vehicle/engage"),
            ("output/npc_simulator_engage", "/simulation/npc_simulator/engage"),  # noqa: E501
            ("output/limit_velocity", "/planning/scenario_planning/max_velocity"),  # noqa: E501
            ("output/object_info", "/simulation/npc_simulator/object_info"),
            ("output/traffic_detection_result", "/perception/traffic_light_recognition/traffic_light_states"),  # noqa: E501
            ("output/lane_change_permission", "/planning/scenario_planning/lane_driving/lane_change_approval"),  # noqa: E501
            # rosparam: add simulator noise or not
            ("rosparam/add_simulator_noise", "/simple_planning_simulator/add_measurement_noise"),  # noqa: E501
            # rosparam: std of simulator pos noise
            ("rosparam/simulator_pos_noise", "/simple_planning_simulator/pos_noise_stddev"),  # noqa: E501
            # rosparam: max velocity
            ("rosparam/max_velocity", "/planning/scenario_planning/motion_velocity_optimizer/max_velocity"),  # noqa: E501
        ],
        # sigterm_timeout=???, TODO
    )

    # When the scenario_runner exits, all nodes should be shut down.
    def on_exit(event: Event, context: LaunchContext):
        return Shutdown(reason="Simulation complete")

    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=scenario_runner,
            on_exit=on_exit
        )
    )

    scenario_api = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=launch_path, parser=Parser()),  # noqa: E501
        launch_arguments=[
            (key, val) for key, val in included_launch_file_args.items()
        ]
    )

    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        output='screen'
    )

    return LaunchDescription([
        scenario_runner,
        shutdown_handler,
        scenario_api,
        rosbag_record,
    ])
