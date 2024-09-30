import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='wall_follow',
            executable='wall_follow_node',
            name='track_node'),
  ])
