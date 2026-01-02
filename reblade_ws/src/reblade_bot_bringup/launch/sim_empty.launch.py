import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Packages
    bringup_pkg = get_package_share_directory("reblade_bot_bringup")
    desc_pkg = get_package_share_directory("reblade_bot_description")
    worlds_pkg = get_package_share_directory("reblade_worlds")

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="empty_with_sensors.sdf",
        description="World SDF filename inside reblade_worlds/worlds/"
    )

    enable_avoidance_arg = DeclareLaunchArgument(
        "enable_avoidance",
        default_value="true",
        description="Enable reactive obstacle avoidance node"
    )

    gazebo_gui_arg = DeclareLaunchArgument(
        "gazebo_gui",
        default_value="false",
        description="Run Gazebo with GUI (for screenshots). Default is headless."
    )

    # Files
    xacro_path = os.path.join(desc_pkg, "urdf", "reblade_bot.urdf.xacro")

    # World full path as a substitution
    world_full_path = PathJoinSubstitution([
        TextSubstitution(text=os.path.join(worlds_pkg, "worlds")),
        LaunchConfiguration("world"),
    ])

    # Robot description
    robot_description = ParameterValue(
        Command(["xacro", " ", xacro_path]),
        value_type=str
    )

    # Common env (VMware-safe)
    gz_env = {
        "LIBGL_ALWAYS_SOFTWARE": "1",
        "QT_QPA_PLATFORM": "xcb",
        "GZ_RENDER_ENGINE": "ogre2",
        "IGN_RENDER_ENGINE": "ogre2",
    }

    # Gazebo headless (default)
    gazebo_headless = ExecuteProcess(
        condition=UnlessCondition(LaunchConfiguration("gazebo_gui")),
        cmd=[
            "ros2", "launch", "ros_gz_sim", "gz_sim.launch.py",
            ["gz_args:=", "-r -s -v 4 ", world_full_path]
        ],
        output="screen",
        additional_env=gz_env,
    )

    # Gazebo with GUI (for screenshots)
    gazebo_gui = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("gazebo_gui")),
        cmd=[
            "ros2", "launch", "ros_gz_sim", "gz_sim.launch.py",
            ["gz_args:=", "-r -v 4 ", world_full_path]
        ],
        output="screen",
        additional_env=gz_env,
    )

    # robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "robot_description": robot_description,
        }],
    )
    tf_aliases = Node(
    package="reblade_tools",
    executable="tf_aliases",
    output="screen",
    parameters=[{
        "use_sim_time": True,
        "aliases": [
            "lidar_link,reblade_bot/lidar_link",
            "camera_link,reblade_bot/camera_link",
            "base_link,reblade_bot/base_link",
        ]
    }],
    )  

    # Spawn robot
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "reblade_bot",
            "-topic", "robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.08",
        ],
    )

    # Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
        ],
        parameters=[{"use_sim_time": True}],
    )

    # Odom -> TF (publishes odom->base_link by stripping prefix)
    odom_to_tf = Node(
    package="reblade_tools",
    executable="odom_to_tf",
    output="screen",
    parameters=[{
        "odom_topic": "/odom",
        "publish_rate_hz": 30.0,
        "use_sim_time": True,
    }],
    respawn=True,
    respawn_delay=2.0,
    )

    # Wheel joint states for RViz (visual only)
    wheel_js = Node(
        package="reblade_tools",
        executable="wheel_joint_state_from_cmdvel",
        output="screen",
        parameters=[{
            "wheel_radius": 0.06,
            "wheel_separation": 0.22,
            "left_joint_name": "left_wheel_joint",
            "right_joint_name": "right_wheel_joint",
            "publish_rate_hz": 30.0,
            "use_sim_time": True,
        }],
    )
    '''
    # Reactive avoidance
    avoidance = Node(
        package="reblade_tools",
        executable="reactive_avoidance",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("enable_avoidance")),
    )
    '''
    navlite = Node(
    package="reblade_tools",
    executable="goal_nav_avoid",
    output="screen",
    parameters=[{
    "use_sim_time": True,
    "forward_speed": 0.18,
    "max_turn": 0.9,
    "goal_tolerance": 0.35,
    # Replace these if you're using the improved hysteresis version:
    "enter_avoid_dist": 0.75,
    "exit_avoid_dist": 0.95,
    "front_arc_deg": 55,
    "avoid_forward_speed": 0.10,
    "avoid_turn_gain": 2.5,
    "cmd_smoothing_alpha": 0.25,
    "yaw_gain": 1.6,
    "waypoints": [2.0, 0.0, 4.0, 0.0, 6.0, 0.0, 0.0, 0.0],
    }],
    condition=IfCondition(LaunchConfiguration("enable_avoidance")),
    )
    
    scan_fix = Node(
    package="reblade_tools",
    executable="scan_frame_fix",
    output="screen",
    parameters=[{"use_sim_time": True}],
    )
  
  
   
    
    
    # RViz
    rviz_config_path = os.path.join(bringup_pkg, "rviz", "reblade.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path] if os.path.exists(rviz_config_path) else [],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        world_arg,
        enable_avoidance_arg,
        gazebo_gui_arg,
        gazebo_headless,
        gazebo_gui,
        robot_state_publisher,
        tf_aliases,
        spawn,
        bridge,
        odom_to_tf,
        wheel_js,
        navlite,
        #avoidance,
        scan_fix,
        rviz,
    ])

