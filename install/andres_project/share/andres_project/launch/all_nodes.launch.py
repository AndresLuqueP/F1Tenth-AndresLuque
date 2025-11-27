from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="andres_project",
            executable="temporizador",
            output="screen",
        ),
        Node(
            package="andres_project",
            executable="lap_trigger",
            output="screen",
        ),
        Node(
            package="andres_project",
            executable="proyectoWF",
            output="screen",
        ),
        Node(
            package="andres_project",
            executable="proyectoRob",
            output="screen",
        ),
        Node(
            package="andres_project",
            executable="fast_ftg",
            output="screen",
        ),
    ])

