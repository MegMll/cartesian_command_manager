from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression


def generate_launch_description():
    # --------------------------------------------------------------------------
    # Configuration & Arguments
    # --------------------------------------------------------------------------
    gui = LaunchConfiguration("gui")
    use_simulation = LaunchConfiguration("use_simulation")

    use_actuator_interface = PythonExpression([
            "'false' if '", use_simulation, "' == 'true' else 'true'"
        ])
    declared_arguments = [
        DeclareLaunchArgument(
            "gui", 
            default_value="true", 
            description="Start RViz2 automatically with this launch file."
        ),
        DeclareLaunchArgument(
            "use_simulation", 
            default_value="false", 
            description="Whether to launch the Gazebo simulation environment"
        ),
    ]

    # --------------------------------------------------------------------------
    # File Paths & Substitutions
    # --------------------------------------------------------------------------
    # Config Files   
    velocity_config = PathJoinSubstitution([
        FindPackageShare("cartesian_command_manager"), "config", "explorer_params.yaml"
    ])

    robot_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("explorer_bringup"), "/launch/simulation_base.launch.py"]),
        launch_arguments={
            'use_POC2': "true",
            'gui': gui,
            'use_sim_time': use_simulation,
            'rviz_delay': '3.0',
            'extra_controllers_config': velocity_config, 
            'use_custom_controllers': "true"
        }.items(),
        condition=IfCondition(use_simulation)
    )

    robot_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("explorer_bringup"), "/launch/hardware_base.launch.py"]),
        launch_arguments={
            'gui': gui,
            'use_sim_time': use_simulation,
            'use_actuator_interface': use_actuator_interface,
            'can_port': "can0",
            'host_id': "45",
            'use_POC2': "true",
            'rviz_delay': '3.0', 
            'extra_controllers_config': velocity_config,
            'use_custom_controllers': "true"

        }.items(),
        condition=UnlessCondition(use_simulation) 
    )

    # --------------------------------------------------------------------------
    # Controllers spawner
    # --------------------------------------------------------------------------
    spawner_qontrol = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=["qontrol_explorer", "--controller-manager", "/controller_manager"],
    )

    spawner_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # --------------------------------------------------------------------------
    # Other Nodes
    # --------------------------------------------------------------------------
    manager_node = Node(
        package="cartesian_command_manager",
        executable="cartesian_command_manager_node",
        name="cartesian_command_manager",
        output="screen",
        parameters=[velocity_config],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
    )

    joystick_mapper_node = Node(
        package="joystick_command_mapper",
        executable="joystick_command_mapper_node",
        name="joystick_command_mapper",
        output="screen",
        parameters=[velocity_config],
    )

    # --------------------------------------------------------------------------
    # Event Handlers
    # --------------------------------------------------------------------------
    delayed_spawner_qontrol = TimerAction(
        period=2.0,
        actions=[spawner_qontrol]
    )
        # --------------------------------------------------------------------------
    # Launch Description
    # --------------------------------------------------------------------------
    nodes_to_start = [
        robot_simulation,
        robot_hardware,
        delayed_spawner_qontrol,
        spawner_gripper_controller,
        manager_node,
        joy_node,
        joystick_mapper_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
