from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Shutdown
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    rname = LaunchConfiguration("rname")
    policy_config = LaunchConfiguration("policy_config")

    robot_name = ParameterValue(Command(["echo -n ", rname]), value_type=str)
    gazebo_model_name = ParameterValue(Command(["echo -n ", rname, "_gazebo"]), value_type=str)
    policy_config_value = ParameterValue(Command(["echo -n ", policy_config]), value_type=str)

    node=Node(
        package="rl_sar",
        executable="rl_sim",
        name="rl_sim",
        output="screen",
        emulate_tty=True,
        on_exit=Shutdown(),
        parameters=[{
            "policy_config": policy_config_value,
        }],
    )

    param_node = Node(
        package="demo_nodes_cpp",
        executable="parameter_blackboard",
        name="param_node",
        on_exit=Shutdown(),
        parameters=[{
            "robot_name": robot_name,
            "gazebo_model_name": gazebo_model_name,
            "policy_config": policy_config_value,
        }],
    )
    
    # joy_node = Node(
    #     package='joy',
    #     executable='joy_node',
    #     name='joy_node',
    #     output='screen',
    #     parameters=[{
    #         'deadzone': 0.1,
    #         'autorepeat_rate': 0.0,
    #     }],
    # )

    # joint_state_broadcaster_node = Node(
    #     package="controller_manager",
    #     executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
    #     arguments=["joint_state_broadcaster"],
    #     output="screen",
    # )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rname",
            description="Robot name (e.g., a1, go2)",
            default_value=TextSubstitution(text=""),
        ),
        DeclareLaunchArgument(
            "policy_config",
            description="Policy sub-directory under policy/<robot>/ (e.g., legged_gym, himloco)",
            default_value=TextSubstitution(text=""),
        ),
        LogInfo(msg="Use /rl_sim/debug_key for interactive debug input under ros2 launch."),
        LogInfo(msg="Example: ros2 topic pub --once /rl_sim/debug_key std_msgs/msg/String \"{data: '0'}\""),
        LogInfo(msg="Publish {data: 'shutdown'} to stop rl_sim and let launch cleanly exit all child processes."),
        node,
        param_node,
    ])
