"""Launch Gazebo, TurtleBot3, live SLAM, Nav2 and an optional one-goal mission."""
from pathlib import Path
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from antropi_demo.turtle_style import build_descriptions


def merge(base, overrides):
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge(base[key], value)
        else:
            base[key] = value
    return base


def launch_setup(context):
    share = Path(get_package_share_directory('antropi_demo'))
    tb3 = Path(get_package_share_directory('nav2_minimal_tb3_sim'))
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    slam = Path(get_package_share_directory('slam_toolbox'))
    runtime = tempfile.TemporaryDirectory(prefix='antropi_demo_')
    work = Path(runtime.name)

    # Expand synchronously: Gazebo must never race an unfinished xacro process.
    world = work / 'world.sdf'
    world.write_text(xacro.process_file(str(share / 'worlds/base_world.sdf.xacro')).toxml())
    robot_description, robot_sdf = build_descriptions(tb3, work)
    for name, source in [('nav2', nav2 / 'params/nav2_params.yaml'),
                         ('slam', slam / 'config/mapper_params_online_async.yaml')]:
        config = merge(yaml.safe_load(source.read_text()),
                       yaml.safe_load((share / f'config/{name}_overrides.yaml').read_text()))
        (work / f'{name}.yaml').write_text(yaml.safe_dump(config))

    def include(path, args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                        launch_arguments=args.items())

    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    gz_args = ['gz', 'sim', '-r', '-s', str(world)]
    if headless:
        gz_args.append('--headless-rendering')

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=lambda _: runtime.cleanup())])),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(tb3 / 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(tb3.parent)),
        ExecuteProcess(cmd=gz_args, output='screen'),
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen',
                       condition=IfCondition(LaunchConfiguration('gazebo_gui'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': True, 'robot_description':
                          robot_description}]),
        include(tb3 / 'launch/spawn_tb3.launch.py',
                {'x_pose': '0', 'y_pose': '0', 'z_pose': '0.01', 'yaw': '0',
                 'robot_sdf': str(robot_sdf)}),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='scene_bridge',
             parameters=[{'use_sim_time': True,
                          'config_file': str(share / 'config/scene_bridge.yaml')}]),
        Node(package='antropi_demo', executable='demo_scene.py', output='screen',
             parameters=[{'use_sim_time': True, 'dynamic_obstacle':
                          LaunchConfiguration('dynamic_obstacle').perform(context).lower() == 'true'}]),
        include(slam / 'launch/online_async_launch.py',
                {'use_sim_time': 'true', 'slam_params_file': str(work / 'slam.yaml')}),
        include(nav2 / 'launch/bringup_launch.py', {
            'use_sim_time': 'true', 'use_localization': 'false',
            'serve_static_map': 'false', 'slam': 'false',
            'use_keepout_zones': 'false', 'use_speed_zones': 'false',
            'params_file': str(work / 'nav2.yaml'), 'autostart': 'true',
            'use_composition': 'true'}),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'rviz/demo.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='antropi_demo', executable='run_mission.py',
             parameters=[{'use_sim_time': True}], output='screen',
             condition=IfCondition(LaunchConfiguration('autostart_mission'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='Use EGL rendering for the simulation server'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('dynamic_obstacle', default_value='true',
                              description='Move the red obstacle; false holds it at its initial pose'),
        DeclareLaunchArgument('autostart_mission', default_value='false',
                              description='Send the finish goal as soon as SLAM and Nav2 are ready'),
        OpaqueFunction(function=launch_setup),
    ])
