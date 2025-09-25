#!/usr/bin/env python3
"""
ROS2 Launch File: G1 Robot Visualization and Control Setup
==========================================================

This launch file demonstrates:
1. ROS2 Python launch system usage
2. Robot state publisher configuration
3. Joint state publisher setup
4. RViz visualization configuration
5. Launch argument handling and conditional node launching

Educational Topics Covered:
- Launch file structure and organization
- Node configuration and parameter passing
- Conditional execution with launch arguments
- File path handling and package resource location
- Robot description (URDF) loading and publishing
- Visualization tool integration (RViz)

Usage Examples:
- With only RViz: ros2 launch g1_manipulation spawn_g1.launch.py
- With joint GUI: ros2 launch g1_manipulation spawn_g1.launch.py jsp_gui:=true
- With zero joint state publisher: ros2 launch g1_manipulation spawn_g1.launch.py jsp:=false
- Without RViz: ros2 launch g1_manipulation spawn_g1.launch.py use_rviz:=false
"""

# Standard Python library imports
import os                                    # Operating system interface for file paths

# ROS2 Launch System Imports
from launch import LaunchDescription          # Main launch description container
from launch.actions import DeclareLaunchArgument  # For command-line arguments
from launch.conditions import IfCondition, UnlessCondition  # Conditional execution
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution  # Dynamic values
from launch_ros.actions import Node          # ROS2 node launch action
from launch_ros.substitutions import FindPackageShare  # Package resource location

def generate_launch_description():
    """
    Launch Description Generator Function
    ====================================
    
    This function creates and returns a LaunchDescription object that defines:
    1. Launch arguments (command-line parameters)
    2. File paths and resource locations
    3. Node configurations and parameters
    4. Conditional execution logic
    
    This is the standard entry point for ROS2 Python launch files.
    """
    
    # =================================================================
    # PACKAGE AND FILE CONFIGURATION
    # =================================================================
    
    # Define package and file names
    # These should match your actual package structure
    package_name = 'g1_manipulation'         # Name of this ROS2 package
    urdf_filename = 'g1.urdf'               # Robot description file
    rviz_config_filename = 'g1.rviz'        # RViz configuration file

    # =================================================================
    # DYNAMIC PATH RESOLUTION
    # =================================================================
    
    # Find package installation directory
    # This works regardless of where ROS2 is installed
    pkg_share_description = FindPackageShare(package_name)
    
    # Build paths to resource files using path substitution
    # PathJoinSubstitution creates paths that work on any operating system
    default_urdf_model_path = PathJoinSubstitution([
        pkg_share_description,    # Package share directory
        'urdf',                   # Subdirectory name
        urdf_filename             # File name
    ])
    
    default_rviz_config_path = PathJoinSubstitution([
        pkg_share_description,    # Package share directory
        'rviz',                   # Subdirectory name
        rviz_config_filename      # File name
    ])

    # =================================================================
    # LAUNCH CONFIGURATION VARIABLES
    # =================================================================
    
    # Create launch configuration variables
    # These can be set from command line: ros2 launch package file.py param:=value
    jsp = LaunchConfiguration('jsp')                    # Joint state publisher flag
    jsp_gui = LaunchConfiguration('jsp_gui')            # Joint state publisher GUI flag
    rviz_config_file = LaunchConfiguration('rviz_config_file')  # RViz config path
    use_rviz = LaunchConfiguration('use_rviz')          # RViz enable flag

    # =================================================================
    # LAUNCH ARGUMENT DECLARATIONS
    # =================================================================
    
    # Declare command-line arguments with defaults and descriptions
    # Users can override these: ros2 launch pkg file.py arg:=value
    
    declare_jsp_cmd = DeclareLaunchArgument(
        name='jsp',                          # Argument name
        default_value='false',               # Default value if not specified
        choices=['true', 'false'],           # Valid values (validation)
        description='Flag to enable joint state publisher'  # Help text
    )

    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui',                      # Argument name
        default_value='false',               # Default value
        choices=['true', 'false'],           # Valid options
        description='Flag to enable joint state publisher GUI'  # Help text
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',             # Argument name
        default_value=default_rviz_config_path,  # Use calculated default path
        description='Path to the RVIZ config file'  # Help text
    )

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',                   # Argument name
        default_value=default_urdf_model_path,  # Use calculated default path
        description='Path to the URDF file to load'  # Help text
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',                     # Argument name
        default_value='true',                # Enable RViz by default
        choices=['true', 'false'],           # Valid options
        description='Whether to launch RViz'  # Help text
    )

    # =================================================================
    # ROBOT DESCRIPTION LOADING
    # =================================================================
    
    # Load URDF file content directly for robot_state_publisher
    # This is more reliable than passing file paths to the node
    robot_description_content = None
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, 'urdf', urdf_filename)
    
    # Read URDF file content into memory
    with open(urdf_model_path, 'r') as infp:
        robot_description_content = infp.read()

    # =================================================================
    # NODE CONFIGURATIONS
    # =================================================================

    # Robot State Publisher Node
    # Publishes robot's kinematic tree and transforms
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',     # ROS2 package name
        executable='robot_state_publisher',  # Executable name within package
        name='robot_state_publisher',        # Node instance name (must be unique)
        output='screen',                     # Show output in terminal
        parameters=[{
            'robot_description': robot_description_content  # URDF content as parameter
        }]
    )

    # Joint State Publisher Node (Basic)
    # Publishes joint states for joints not controlled by other nodes
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',     # ROS2 package name
        executable='joint_state_publisher',  # Executable name
        name='joint_state_publisher',        # Node instance name
        condition=IfCondition(jsp)           # Only launch if jsp argument is true
    )

    # Joint State Publisher GUI Node
    # Provides interactive sliders for manually controlling joints
    start_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui', # ROS2 package name
        executable='joint_state_publisher_gui',  # Executable name
        name='joint_state_publisher_gui',    # Node instance name
        condition=IfCondition(jsp_gui)       # Only launch if jsp_gui argument is true
    )

    # RViz Visualization Node
    # 3D visualization tool for robots and sensor data
    start_rviz_cmd = Node(
        package='rviz2',                     # ROS2 package name (RViz version 2)
        executable='rviz2',                  # Executable name
        name='rviz2',                        # Node instance name
        output='screen',                     # Show output in terminal
        arguments=['-d', rviz_config_file],  # Command line arguments: -d <config_file>
        condition=IfCondition(use_rviz)      # Only launch if use_rviz argument is true
    )

    # =================================================================
    # LAUNCH DESCRIPTION ASSEMBLY
    # =================================================================

    # Create the main launch description container
    ld = LaunchDescription()

    # Add launch argument declarations first
    # These must be added before any nodes that use the arguments
    ld.add_action(declare_jsp_cmd)
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add node launch actions
    # Order matters for dependencies - robot_state_publisher should start first
    ld.add_action(start_robot_state_publisher_cmd)    # Always launched (core functionality)
    ld.add_action(start_joint_state_publisher_cmd)    # Conditional (jsp argument)
    ld.add_action(start_joint_state_publisher_gui_cmd) # Conditional (jsp_gui argument)
    ld.add_action(start_rviz_cmd)                     # Conditional (use_rviz argument)

    # Return the complete launch description
    return ld