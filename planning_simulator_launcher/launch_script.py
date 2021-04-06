from pathlib import Path

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml import Parser

from launch import LaunchContext, LaunchDescription
from launch.actions import (ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler, Shutdown, DeclareLaunchArgument)
from launch.event import Event
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import launch


def launch_description(*, launch_path, vehicle_model,
                       scenario_runner_args, included_launch_file_args,
                       record_rosbag, log_save_dir):
    vehicle_package = f'{vehicle_model}_description'
    vehicle_package_path = FindPackageShare(
        vehicle_package).find(vehicle_package)
    vehicle_info_param_path = Path(
        vehicle_package_path) / 'config' / 'vehicle_info.param.yaml'
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
            ("input/engage", "/simulation/npc_simulator/engage"),
            ("input/ego_vehicle_pose", "/current_pose"),
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
            ("output/dynamic_object_info",
     "/simulation/dummy_perception_publisher/object_info"),
            ("output/debug_object_info",
     "/simulation/npc_simulator/ground_truth_object_info")
        ],
        # sigterm_timeout=???, TODO
    )

    psim = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=launch_path, parser=Parser()),  # noqa: E501
        launch_arguments=[
            (key, val) for key, val in included_launch_file_args.items()
        ]
    )

    # shutdown other processes when scenario_runner has died
    shutdown_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=scenario_runner,
            on_exit=[
                launch.actions.EmitEvent(event=launch.events.Shutdown()),
            ]
        )
    )

    record_condition = LaunchConfiguration(
        'record_rosbag', default=record_rosbag)
    rosbag_record = ExecuteProcess(
        cmd=['ros2',
             'bag',
             'record',
             '-a',
             '-b',
             str(1024 * 1024 * 1024 * 2),
             '--compression-mode',
             'file',
             '--compression-format',
             'zstd',
             '-o',
             log_save_dir],
        output='screen',
        sigterm_timeout=LaunchConfiguration(
            'sigterm_timeout', default=30),
        sigkill_timeout=LaunchConfiguration(
            'sigkill_timeout', default=30),
        condition=IfCondition(record_condition)
    )
    # maximum bag size: 10G

    return LaunchDescription([
        scenario_runner,
        psim,
        rosbag_record,
        shutdown_handler,
    ])
