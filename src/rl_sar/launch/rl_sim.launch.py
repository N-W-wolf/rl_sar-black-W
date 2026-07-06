from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Shutdown
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    rname = LaunchConfiguration("rname")
    policy_config = LaunchConfiguration("policy_config")

    node=Node(
        package="rl_sar",
        executable="rl_sim",
        name="rl_sim",
        output="screen",
        emulate_tty=True,
        on_exit=Shutdown(),
        parameters=[{
            "robot_name": rname,
            "policy_config": policy_config,
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
            description="Robot name (e.g., black, a1, go2)",
            default_value=TextSubstitution(text="black"),
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
    ])
