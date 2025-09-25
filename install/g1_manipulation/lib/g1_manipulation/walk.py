#!/usr/bin/env python3

"""
ROS2 Python Node: Left Arm Waving Motion Controller
==================================================

This file demonstrates:
1. Creating a ROS2 Python node
2. Publishing JointState messages for robot motion control
3. Using timers for periodic execution
4. Implementing smooth motion using sine waves
5. Working with robot joint states

Educational Topics Covered:
- ROS2 Node class inheritance
- Publisher/Timer patterns
- Message structure (JointState)
- Mathematical motion generation
- Robotics coordinate systems
"""

# Standard ROS2 Python imports
import rclpy                           # Main ROS2 Python client library
from rclpy.node import Node            # Base class for creating ROS2 nodes
from sensor_msgs.msg import JointState # Message type for robot joint information
import math                            # For sine wave calculations

class JointStatePublisher(Node):
    """
    ROS2 Node Class for Publishing Joint States
    ==========================================
    
    This class inherits from rclpy.Node and creates a publisher that sends
    joint position commands to make the robot's left arm wave.
    
    Key Concepts:
    - Node inheritance: Extends rclpy.Node for ROS2 functionality
    - Publisher pattern: Sends messages to other nodes
    - Timer callbacks: Executes functions at regular intervals
    - Joint state control: Commands robot joint positions
    """

    def __init__(self):
        """
        Node Constructor - Initialize the ROS2 node
        ==========================================
        
        This method sets up:
        1. Node name and ROS2 connection
        2. Publisher for joint state messages
        3. Timer for periodic message publishing
        4. Motion parameters for waving animation
        """
        # Call parent constructor with unique node name
        # Node names must be unique in the ROS2 system
        super().__init__('my_legs_joint_state_publisher')
        
        # Create a publisher for JointState messages
        # Topic: '/joint_states' - standard topic for robot joint information
        # Queue size: 10 - buffers up to 10 messages if subscriber is slow
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        
        # Create a timer for smooth motion control
        # 0.02 seconds = 50Hz frequency for smooth visualization
        # Higher frequency = smoother motion but more CPU usage
        # Lower frequency = less CPU usage but less smooth motion
        self.timer = self.create_timer(0.02, self.publish_joint_states) 
        
        # Log node startup (appears in terminal/logs)
        self.get_logger().info('Joint state publisher node started.')
        
        # Initialize motion control variables
        self.time_elapsed = 0.0      # Tracks time for sine wave calculation
        self.wave_period = 4.0       # Complete wave cycle duration in seconds

    def publish_joint_states(self):
        """
        Timer Callback Function - Called every 0.02 seconds (50Hz)
        =========================================================
        
        This function:
        1. Creates a JointState message
        2. Fills in joint names and positions
        3. Calculates smooth waving motion using sine waves
        4. Publishes the message to control the robot
        5. Updates timing for next iteration
        """
        # Create new JointState message
        msg = JointState()
        
        # Set message header with timestamp and coordinate frame
        msg.header.stamp = self.get_clock().now().to_msg()  # Current ROS time
        msg.header.frame_id = ''                            # No specific frame
        
        # Define all robot joint names
        # These names must match the URDF joint definitions
        msg.name = [
            # Lower body joints (legs)
            'left_hip_pitch_joint',      # Left leg hip forward/backward
            'left_hip_roll_joint',       # Left leg hip side-to-side  
            'left_hip_yaw_joint',        # Left leg hip rotation
            'left_knee_joint',           # Left leg knee bend
            'left_ankle_pitch_joint',    # Left foot up/down
            'left_ankle_roll_joint',     # Left foot tilt
            'right_hip_pitch_joint',     # Right leg hip forward/backward
            'right_hip_roll_joint',      # Right leg hip side-to-side
            'right_hip_yaw_joint',       # Right leg hip rotation
            'right_knee_joint',          # Right leg knee bend
            'right_ankle_pitch_joint',   # Right foot up/down
            'right_ankle_roll_joint',    # Right foot tilt
            
            # Torso joint
            'waist_yaw_joint',           # Waist rotation
            
            # Left arm joints (the ones we'll animate)
            'left_shoulder_pitch_joint',  # Left shoulder forward/backward
            'left_shoulder_roll_joint',   # Left shoulder up/down (main waving motion)
            'left_shoulder_yaw_joint',    # Left shoulder rotation
            'left_elbow_joint',          # Left elbow bend
            'left_wrist_roll_joint',     # Left wrist rotation
            'left_wrist_pitch_joint',    # Left wrist up/down
            'left_wrist_yaw_joint',      # Left wrist side rotation
            'left_hand_connector_joint', # Left hand connection
            'left_hand_left_finger_joint',  # Left hand finger
            'left_hand_right_finger_joint', # Left hand finger
            
            # Right arm joints (kept at zero for this demo)
            'right_shoulder_pitch_joint',    # Right shoulder forward/backward
            'right_shoulder_roll_joint',     # Right shoulder up/down
            'right_shoulder_yaw_joint',      # Right shoulder rotation
            'right_elbow_joint',             # Right elbow bend
            'right_wrist_roll_joint',        # Right wrist rotation
            'right_wrist_pitch_joint',       # Right wrist up/down
            'right_wrist_yaw_joint',         # Right wrist side rotation
            'right_hand_connector_joint',    # Right hand connection
            'right_hand_left_finger_joint',  # Right hand finger
            'right_hand_right_finger_joint'  # Right hand finger
        ]

        # Initialize all joint positions to zero (neutral pose)
        # This creates the "base pose" before adding waving motion
        base_positions = [
            0.0,          # left_hip_pitch_joint
            0.0,          # left_hip_roll_joint
            0.0,          # left_hip_yaw_joint
            0.0,          # left_knee_joint
            0.0,          # left_ankle_pitch_joint
            0.0,          # left_ankle_roll_joint
            0.0,          # right_hip_pitch_joint
            0.0,          # right_hip_roll_joint
            0.0,          # right_hip_yaw_joint
            0.0,          # right_knee_joint
            0.0,          # right_ankle_pitch_joint
            0.0,          # right_ankle_roll_joint
            0.0,          # waist_yaw_joint
            0.0,          # left_shoulder_pitch_joint (index 13)
            0.0,          # left_shoulder_roll_joint (index 14) 
            0.0,          # left_shoulder_yaw_joint (index 15)
            0.0,          # left_elbow_joint (index 16)
            0.0,          # left_wrist_roll_joint (index 17)
            0.0,          # left_wrist_pitch_joint (index 18)
            0.0,          # left_wrist_yaw_joint (index 19)
            0.0,          # left_hand_connector_joint
            0.0,          # left_hand_left_finger_joint
            0.0,          # left_hand_right_finger_joint
            0.0,          # right_shoulder_pitch_joint
            0.0,          # right_shoulder_roll_joint
            0.0,          # right_shoulder_yaw_joint
            0.0,          # right_elbow_joint
            0.0,          # right_wrist_roll_joint
            0.0,          # right_wrist_pitch_joint
            0.0,          # right_wrist_yaw_joint
            0.0,          # right_hand_connector_joint
            0.0,          # right_hand_left_finger_joint
            0.0           # right_hand_right_finger_joint
        ]

        # Copy base positions to message (start with neutral pose)
        msg.position = base_positions.copy()

        # MOTION GENERATION: Create smooth waving motion using sine waves
        # =============================================================
        
        # Motion parameters
        wave_amplitude = 1.0                                    # Maximum angle in radians (~28.6 degrees)
        wave_frequency = 2.0 * math.pi / self.wave_period      # Angular frequency (rad/s)
        
        # Calculate primary wave position using sine function
        # sin() creates smooth oscillation between -1 and +1
        wave_position = wave_amplitude * math.sin(wave_frequency * self.time_elapsed)
        
        # Apply coordinated motion to joints
        # Each joint gets different motion for natural waving
        
        # Hip pitch
        # Index 0: left_hip_pitch_joint
        # Index 6: right_hip_pitch_joint
        msg.position[0] = +0.25 * wave_position
        msg.position[6] = -0.25 * wave_position

        # Knee
        # Index 3: left_knee_joint
        # Index 9: right_knee_joint
        msg.position[3] = 0.3 + 0.25 * math.sin(wave_frequency * self.time_elapsed - math.pi / 3)
        msg.position[9] = 0.3 - 0.25 * math.sin(wave_frequency * self.time_elapsed - math.pi / 3)
        
        # Shoulder pitch
        # Index 13: left_shoulder_pitch_joint
        # Index 23: right_shoulder_pitch_joint
        msg.position[13] = -0.4 * wave_position
        msg.position[23] = +0.4 * wave_position
        
        # Elbow
        # Index 16: left_elbow_joint
        # Index 26: right_elbow_joint
        msg.position[16] = 1.3 - 0.1 * wave_position
        msg.position[26] = 1.3 + 0.1 * wave_position

        # JointState message can include velocity and effort, but we leave empty
        # Empty arrays tell the robot to ignore velocity/force control
        msg.velocity = []    # Joint velocities (empty = not controlled)
        msg.effort = []      # Joint forces/torques (empty = not controlled)

        # Publish the message to the ROS2 system
        # This sends the joint commands to any listening nodes
        self.publisher_.publish(msg)
        
        # Update time for next iteration
        # This advances the sine wave for smooth continuous motion
        self.time_elapsed += 0.02  # 20ms increment (matches timer period)
        
        # Periodic logging to monitor the system (every 2 seconds)
        # Modulo operation prevents log spam while showing progress
        if int(self.time_elapsed * 10) % 20 == 0:
            self.get_logger().info(f'Published smooth joint state - Wave position: {wave_position:.3f}')


def main(args=None):
    """
    Main Function - Entry point for ROS2 node execution
    ==================================================
    
    This function:
    1. Initializes the ROS2 Python client library
    2. Creates and starts the node
    3. Keeps the node running until shutdown
    4. Cleans up resources
    
    This is the standard pattern for ROS2 Python nodes.
    """
    # Initialize ROS2 Python client library
    # Must be called before creating any ROS2 objects
    rclpy.init(args=args)
    
    # Create an instance of our custom node class
    joint_state_publisher = JointStatePublisher()
    
    # Keep the node running and processing callbacks
    # This blocks until the node is shut down (Ctrl+C)
    rclpy.spin(joint_state_publisher)
    
    # Clean shutdown: destroy node and cleanup ROS2
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

# Python idiom: only run main() if this file is executed directly
# (not when imported as a module)
if __name__ == '__main__':
    main()
