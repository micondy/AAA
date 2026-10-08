from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bag = LaunchConfiguration("bag")
    rate = LaunchConfiguration("rate")
    player = ExecuteProcess(
        cmd=["ros2", "bag", "play", bag, "--clock", "--rate", rate],
        output="screen",
        sigterm_timeout="10",
        sigkill_timeout="5",
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "bag", description="Existing camera rosbag directory to replay"
        ),
        DeclareLaunchArgument("rate", default_value="1.0"),
        player,
    ])
