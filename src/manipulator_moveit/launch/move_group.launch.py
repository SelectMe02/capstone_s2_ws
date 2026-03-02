import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Robot description (URDF from xacro)
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory('manipulator'),
            'description',
            'capstone_manipulator.urdf.xacro',
        )
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Robot semantic description (SRDF)
    robot_description_semantic_path = os.path.join(
        get_package_share_directory('manipulator_moveit'),
        'config',
        'manipulator.srdf',
    )
    with open(robot_description_semantic_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # Kinematics (kinematics.yaml)
    kinematics_yaml_path = os.path.join(
        get_package_share_directory('manipulator_moveit'),
        'config',
        'kinematics.yaml',
    )
    with open(kinematics_yaml_path, 'r') as f:
        kinematics_yaml = yaml.safe_load(f)

    robot_description_kinematics = {
        'robot_description_kinematics': kinematics_yaml
    }

    # Planning pipeline (OMPL)
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
    with open(ompl_planning_yaml_path, 'r') as f:
        ompl_planning_yaml = yaml.safe_load(f)
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory execution
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    # MoveIt controllers (moveit_controllers.yaml)
    moveit_controllers_yaml_path = os.path.join(
        get_package_share_directory('manipulator_moveit'),
        'config',
        'moveit_controllers.yaml',
    )
    with open(moveit_controllers_yaml_path, 'r') as f:
        moveit_simple_controllers_yaml = yaml.safe_load(f)

    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    # Planning scene monitor
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
    }

    # move_group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    ld = LaunchDescription()
    ld.add_action(move_group_node)
    return ld