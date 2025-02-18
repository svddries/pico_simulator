import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
import launch_ros.actions

def generate_launch_description():
    # Get the package directory dynamically
    emc_simulator_share_dir = get_package_share_directory("emc_simulator")

    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=os.path.join(emc_simulator_share_dir, "data", "defaultconfig.json"),
        description="Path to the simulation config file"
    )

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(emc_simulator_share_dir, "data", "heightmap_metadata.yaml"),
        description="Path to the map file"
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[{"yaml_filename": LaunchConfiguration("map")}]
    )

    simulator = Node(
        package="emc_simulator",
        executable="pico_simulator",
        name="simulator_node",
        arguments=["--config", LaunchConfiguration("config")],
        output="screen",
        parameters=[os.path.join(emc_simulator_share_dir, "simconfig", "simbot_config.yaml")]
    )

    lifecycle_nodes = ['map_server']
    use_sim_time = True
    autostart = True

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]
    )

    LaunchDescription().add_action(map_server)
    LaunchDescription().add_action(start_lifecycle_manager_cmd)


    return LaunchDescription([
        config_arg,
        map_arg,
        # robot_state_publisher,
        # joint_state_publisher,
        map_server,
        simulator,
        start_lifecycle_manager_cmd
    ])