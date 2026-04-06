from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node

from launch.actions import TimerAction

def generate_launch_description():

    initial_pose_publisher = Node(
        package='template_package',
        executable='publish_initial_pose',
        name='initial_pose_publisher'
    )

    navigate_to_object_area = Node(
        package='template_package',
        executable='navigate_to_pose',
        name='navigate_to_pose_node',
        parameters=[{
            'x': 1.30,
            'y': -0.2,
            'yaw': -0.9
        }]
    )

    # wait 10s to have initial pose actually received by nav2
    navigate_after_initial_pose = TimerAction(
        period=10.0,
        actions=[navigate_to_object_area]
    )

    """ navigate_after_initial_pose = RegisterEventHandler(
        OnProcessExit(
            target_action=initial_pose_publisher,
            on_exit=[navigate_to_object_area],
        )
    ) """

    aruco_detection = Node(
        package='template_package',
        executable='read_marker_position',
        name='read_marker_pose'
    )

    detect_after_navigating = RegisterEventHandler(
        OnProcessExit(
            target_action=navigate_to_object_area,
            on_exit=[aruco_detection],
        )
    )

    park_close_to_marker = Node(
        package='template_package',
        executable='follow_marker_pose',
        name='follow_marker'
    )

    park_after_detection_started = RegisterEventHandler(
        OnProcessStart(
            target_action=aruco_detection,
            on_start=park_close_to_marker,
        )
    )

    pick = Node(
        package='template_package',
        executable='execute_pick',
        name='pick_node'
    )

    pick_after_parking = RegisterEventHandler(
        OnProcessExit(
            target_action=park_close_to_marker,
            on_exit=[pick],
        )
    )

    navigate_to_object_placing_area = Node(
        package='template_package',
        executable='navigate_to_pose',
        name='navigate_to_pose_node',
        parameters=[{
            'x': 2.1,
            'y': 1.9,
            'yaw': 3.0
        }]
    )

    navigate_to_placing_area_after_pick = RegisterEventHandler(
        OnProcessExit(
            target_action=pick,
            on_exit=[navigate_to_object_placing_area],
        )
    )

    # not the finest approach to only use sim time here, but it is
    # necessary for moveit's getCurrentPosition() method
    place = Node(
        package='template_package',
        executable='execute_place',
        name='place_node',
        parameters=[{
            'use_sim_time': True
        }]
    )

    place_after_navigating_to_placing_area = RegisterEventHandler(
        OnProcessExit(
            target_action=navigate_to_object_placing_area,
            on_exit=[place],
        )
    )

    return LaunchDescription([
        initial_pose_publisher,
        navigate_after_initial_pose,
        detect_after_navigating,
        park_after_detection_started,
        pick_after_parking,
        navigate_to_placing_area_after_pick,
        place_after_navigating_to_placing_area
    ])