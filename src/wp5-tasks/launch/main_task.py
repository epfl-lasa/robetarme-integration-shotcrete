from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the 'taskType' argument with a default value
    task_type_arg = DeclareLaunchArgument(
        'taskType',
        default_value='shotcrete',
        description='Type of task to execute'
    )

    # Define the node with a dynamically generated name
    wp5_task_node = Node(
        package='wp5_tasks',
        executable='wp5_main_task_node',
        name=PythonExpression(["'", LaunchConfiguration('taskType'), "_wp5_main_task_node'"]),  
        output='screen',
        parameters=[{'taskType': LaunchConfiguration('taskType')}]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        task_type_arg,
        wp5_task_node
    ])
