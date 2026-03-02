import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import yaml


def generate_launch_description():

    # Rviz config save file
    rviz_config = os.path.join(
        get_package_share_directory('manipulator_moveit'),
        'config',
        'moveit.rviz'
    )

    # Robot description
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory('manipulator_moveit'),
            'config',
            'manipulator.urdf.xacro',
        )
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Robot description Semantic config
    robot_description_semantic_path = os.path.join(
        get_package_share_directory('manipulator_moveit'),
        'config',
        'manipulator.srdf',
    )
    with open(robot_description_semantic_path, 'r') as file:
        robot_description_semantic_config = file.read()

    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    # Planning Functionality
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ),
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml_path = os.path.join(
        get_package_share_directory('manipulator_moveit'),
        'config',
        'ompl_planning.yaml',
    )
    with open(ompl_planning_yaml_path, 'r') as file:
        ompl_planning_yaml = yaml.safe_load(file)
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # kinematics yaml
    kinematics_yaml_path = os.path.join(
        get_package_share_directory('manipulator_moveit'),
        'config',
        'kinematics.yaml',
    )
    with open(kinematics_yaml_path, 'r') as file:
        kinematics_yaml = yaml.safe_load(file)

    ld = LaunchDescription()

    warehouse_ros_config = {
        # For warehouse_ros_sqlite
        'warehouse_plugin': 'warehouse_ros_sqlite::DatabaseConnection',
        'warehouse_host': '/home/robotis/warehouse_db.sqlite',  # change to your path
        'port': 33829,
        'scene_name': '',  # If scene name is empty, all scenes will be used
        'queries_regex': '.*',
    }

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            warehouse_ros_config,
        ]
    )

    ld.add_action(rviz_node)

    return ld
