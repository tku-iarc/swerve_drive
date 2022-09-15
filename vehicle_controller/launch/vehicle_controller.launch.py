from email.policy import default
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_context import LaunchContext

def launch_setup(context, *args, **kwargs):

    # General arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    vehicle_data_file = LaunchConfiguration("vehicle_data_file")
    prefix = LaunchConfiguration("prefix")
    robot_type = LaunchConfiguration("robot_type")
    namespace = LaunchConfiguration("namespace")
    sim = LaunchConfiguration("sim")
    use_sim_time = LaunchConfiguration("use_sim_time")
    joy_dev = LaunchConfiguration("joy_dev")

    vehicle_data = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config/" + robot_type.perform(context), vehicle_data_file]
    )

    vehicle_controller_node = Node(
        package="vehicle_controller",
        executable="vehicle_controller_node",
        namespace=namespace,
        parameters=[{"sim": sim, "prefix": prefix, "use_sim_time": use_sim_time}, vehicle_data],
        output="screen"
    )

    joystick_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        namespace=namespace,
        parameters=[{"use_sim_time": use_sim_time, "dev": "/dev/input/" + joy_dev.perform(context)}],
    )

    nodes_to_start = [
        vehicle_controller_node,
        joystick_node,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="vehicle_controller",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "vehicle_data_file",
            default_value="vehicle_data.yaml",
            description="YAML file with the vehicle configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value="true",
            description="Using gazebo or not",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Using sim time or not",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            default_value="yhs_fw01",
            description="mobile base type",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "joy_dev",
            default_value="js1",
            description="port of joy",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace for node"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
