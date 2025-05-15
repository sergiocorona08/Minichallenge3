import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, GroupAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robots = [
        {'name': 'robot1', 'x_pose': 0.5, 'y_pose': 0.5},
        {'name': 'robot2', 'x_pose': -0.5, 'y_pose': 0.5},
    ]

    urdf_file = os.path.join(
        get_package_share_directory('puzzlebot_sim'),
        'urdf',
        'puzzlebot.urdf'
    )
    nodes = []
    
    for robot in robots:
        with open(urdf_file, 'r') as f:
            robot_description = f.read()
        
        group = GroupAction([
            
            Node(
                package='puzzlebot_sim',
                executable='puzzlebot_sim',
                namespace=robot['name'],
                name='puzzlebot_sim',
    
                output='screen',
            ),
            Node(
                package='puzzlebot_sim',
                executable='localisation',
                namespace=robot['name'],
                name='localisation',
                output='screen',
            ),
            #para que las tf sean unicas en cada robot
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=robot['name'],
                parameters=[{
                    'frame_prefix': f"{robot['name']}/",
                    'robot_description': robot_description,
                    
                }],
                arguments=[urdf_file],
                output='screen'
            ),
            Node(
                package='puzzlebot_sim',
                executable='joint_state_publisher',
                namespace=robot['name'],
                name='joint_state_publisher',
                parameters=[{
                    'initial_pose': [robot['x_pose'], robot['y_pose'], 0.0],
                    'odometry_frame': "odom",
                    
                }],
                output='screen',
            ),
            
            
            Node(
                package='puzzlebot_sim',
                executable='point_stabilisation_controller',
                namespace=robot['name'],
                name='point_stabilisation_controller',
                output='screen',
            ),
            
            Node(
                package='puzzlebot_sim',
                executable='shapeDrawer',
                namespace=robot['name'],
                name='shapeDrawer',
                output='screen',
            ),
     
        ])
        nodes.append(group)


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('puzzlebot_sim'), 'rviz', 'multi_robot.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    rqt_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        name='rqt_graph',
        output='screen'
    )
    rqt_tf_tree_node = Node(
        package='rqt_tf_tree',
        executable='rqt_tf_tree',
        name='rqt_tf_tree',
        output='screen',
    )
    
    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        *nodes,
        rviz_node,
        rqt_node,
        rqt_tf_tree_node,
        ])