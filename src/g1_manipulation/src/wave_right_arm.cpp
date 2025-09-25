/*
 * ROS2 C++ Implementation: Right Arm Waving Motion Controller
 * ===========================================================
 * 
 * This source file demonstrates:
 * 1. C++ class implementation and constructor initialization lists
 * 2. ROS2 C++ publisher and timer usage
 * 3. Mathematical motion generation using sine waves
 * 4. JointState message construction and publishing
 * 5. Main function pattern for ROS2 C++ nodes
 * 
 * Educational Topics Covered:
 * - Header inclusion and implementation separation
 * - Constructor initialization lists
 * - Smart pointer usage and memory management
 * - STL containers and algorithms
 * - ROS2 logging and time handling
 * - Mathematical robotics motion control
 */

// Include the corresponding header file
// This provides the class declaration that we're implementing
#include "g1_manipulation/wave_right_arm.hpp"

// C++ chrono literals for time specifications
// Enables convenient time notation like 20ms, 1s, etc.
using namespace std::chrono_literals;

/*
 * NAMESPACE IMPLEMENTATION
 * =======================
 * All code must be within the same namespace as declared in the header
 */
namespace g1_manipulation
{

/*
 * CONSTRUCTOR IMPLEMENTATION
 * =========================
 * Uses initialization list for efficient member variable setup.
 * This is preferred over assignment in the constructor body.
 */
JointStatePublisher::JointStatePublisher()
: Node("my_right_arm_joint_state_publisher"),  // Initialize base class with node name
  time_elapsed_(0.0),                          // Initialize motion timer
  wave_period_(4.0)                            // Set wave cycle duration
{
  /*
   * PUBLISHER SETUP
   * ==============
   * Creates a publisher for JointState messages on the standard topic.
   * Template parameter specifies the message type.
   */
  // Create publisher for robot joint commands
  // Topic: "/joint_states" - standard ROS topic for joint information
  // Queue size: 10 - buffers messages if subscriber processing is slow
  publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  
  /*
   * TIMER SETUP
   * ===========
   * Creates a wall timer that calls our callback function periodically.
   * 20ms = 50Hz frequency provides smooth motion visualization.
   */
  // Create timer for periodic execution (50Hz for smooth motion)
  // std::bind connects the member function to the timer callback
  timer_ = this->create_wall_timer(20ms, std::bind(&JointStatePublisher::publish_joint_states, this));
  
  // Log successful node initialization
  RCLCPP_INFO(this->get_logger(), "Joint state publisher node started.");
  
  /*
   * ROBOT JOINT CONFIGURATION
   * =========================
   * Initialize the complete list of robot joint names.
   */
  // Initialize joint names (must match URDF joint definitions exactly)
  joint_names_ = {
    // Lower body joints - legs and feet
    "left_hip_pitch_joint",        // Index 0:  Left leg hip forward/backward
    "left_hip_roll_joint",         // Index 1:  Left leg hip side-to-side
    "left_hip_yaw_joint",          // Index 2:  Left leg hip rotation
    "left_knee_joint",             // Index 3:  Left leg knee bend
    "left_ankle_pitch_joint",      // Index 4:  Left foot up/down
    "left_ankle_roll_joint",       // Index 5:  Left foot tilt
    "right_hip_pitch_joint",       // Index 6:  Right leg hip forward/backward
    "right_hip_roll_joint",        // Index 7:  Right leg hip side-to-side
    "right_hip_yaw_joint",         // Index 8:  Right leg hip rotation
    "right_knee_joint",            // Index 9:  Right leg knee bend
    "right_ankle_pitch_joint",     // Index 10: Right foot up/down
    "right_ankle_roll_joint",      // Index 11: Right foot tilt
    
    // Torso joint
    "waist_yaw_joint",             // Index 12: Waist rotation
    
    // Left arm joints (kept neutral for this demo)
    "left_shoulder_pitch_joint",   // Index 13: Left shoulder forward/backward
    "left_shoulder_roll_joint",    // Index 14: Left shoulder up/down
    "left_shoulder_yaw_joint",     // Index 15: Left shoulder rotation
    "left_elbow_joint",            // Index 16: Left elbow bend
    "left_wrist_roll_joint",       // Index 17: Left wrist rotation
    "left_wrist_pitch_joint",      // Index 18: Left wrist up/down
    "left_wrist_yaw_joint",        // Index 19: Left wrist side rotation
    "left_hand_connector_joint",   // Index 20: Left hand connection
    "left_hand_left_finger_joint", // Index 21: Left hand finger
    "left_hand_right_finger_joint",// Index 22: Left hand finger
    
    // Right arm joints (the ones we'll animate for waving)
    "right_shoulder_pitch_joint",    // Index 23: Right shoulder forward/backward
    "right_shoulder_roll_joint",     // Index 24: Right shoulder up/down (main waving motion)
    "right_shoulder_yaw_joint",      // Index 25: Right shoulder rotation
    "right_elbow_joint",             // Index 26: Right elbow bend
    "right_wrist_roll_joint",        // Index 27: Right wrist rotation
    "right_wrist_pitch_joint",       // Index 28: Right wrist up/down
    "right_wrist_yaw_joint",         // Index 29: Right wrist side rotation
    "right_hand_connector_joint",    // Index 30: Right hand connection
    "right_hand_left_finger_joint",  // Index 31: Right hand finger
    "right_hand_right_finger_joint"  // Index 32: Right hand finger
  };
  
  /*
   * BASE POSITION INITIALIZATION
   * ============================
   * Create vector with all joints at neutral (zero) position.
   * This provides the starting pose before adding motion.
   */
  // Initialize all joint positions to zero (neutral robot pose)
  // vector constructor: size = joint_names_.size(), all values = 0.0
  base_positions_ = std::vector<double>(joint_names_.size(), 0.0);
}

/*
 * TIMER CALLBACK IMPLEMENTATION
 * =============================
 * This function is called every 20ms (50Hz) to generate and publish
 * smooth waving motion for the robot's right arm.
 */
void JointStatePublisher::publish_joint_states()
{
  /*
   * MESSAGE CREATION AND HEADER SETUP
   * =================================
   * Create a new JointState message and fill in the header information.
   */
  // Create new JointState message instance
  auto msg = sensor_msgs::msg::JointState();
  
  // Set message header with current timestamp
  msg.header.stamp = this->get_clock()->now();  // ROS2 time stamp
  msg.header.frame_id = "";                     // No specific coordinate frame
  
  /*
   * JOINT NAMES AND BASE POSITIONS
   * ==============================
   * Copy the joint names and neutral positions to the message.
   */
  // Set joint names (must match URDF joint definitions)
  msg.name = joint_names_;
  
  // Copy base positions (all zeros) as starting point
  msg.position = base_positions_;
  
  /*
   * MOTION GENERATION: MATHEMATICAL WAVING ANIMATION
   * ===============================================
   * Generate smooth, natural-looking waving motion using sine waves.
   * Different joints get different motion patterns for realistic movement.
   */
  
  // Motion parameters for smooth waving
  double wave_amplitude = 0.5;                           // Maximum joint angle (radians ≈ 28.6°)
  double wave_frequency = 2.0 * M_PI / wave_period_;     // Angular frequency (rad/s)
  
  // Calculate primary wave position using sine function
  // sin() oscillates between -1 and +1, amplitude scales this
  double wave_position = wave_amplitude * sin(wave_frequency * time_elapsed_);
  
  /*
   * RIGHT ARM JOINT MOTION APPLICATION
   * ==================================
   * Apply coordinated motion to right arm joints (indices 23-29).
   * Each joint gets different motion for natural waving appearance.
   * 
   * Key difference from left arm: some angles are negated to mirror
   * the motion appropriately for the right side of the robot.
   */
  
  // Index 23: Right shoulder pitch - slight up/down motion
  // Same as left arm - no mirroring needed for pitch motion
  msg.position[23] = 0.2 * sin(wave_frequency * time_elapsed_);
  
  // Index 24: Right shoulder roll - main waving motion (MIRRORED)
  // Negative values and offset to wave naturally for right arm
  // -0.8 rad offset lowers right arm, sine wave adds waving motion
  msg.position[24] = -0.8 - 0.3 * sin(wave_frequency * time_elapsed_);
  
  // Index 25: Right shoulder yaw - rotation motion (MIRRORED)
  // Negated to rotate in opposite direction from left arm
  msg.position[25] = -wave_position;
  
  // Index 26: Right elbow - faster secondary motion
  // Same pattern as left arm - elbow bending doesn't need mirroring
  // Frequency × 2 creates faster motion, 0.3 rad offset keeps elbow bent
  msg.position[26] = 0.3 + 0.2 * sin(wave_frequency * time_elapsed_ * 2);
  
  // Wrist joints - coordinated motion following the main wave
  msg.position[27] = -wave_position * 0.5;    // Right wrist roll (MIRRORED, 50% amplitude)
  msg.position[28] = wave_position * 0.3;     // Right wrist pitch (30% amplitude)
  msg.position[29] = -wave_position * 0.4;    // Right wrist yaw (MIRRORED, 40% amplitude)
  
  /*
   * MESSAGE COMPLETION AND PUBLISHING
   * =================================
   * Complete the message setup and publish to the ROS2 system.
   */
  
  // Clear velocity and effort arrays (position-only control)
  // Empty arrays tell the robot to ignore velocity/force control
  msg.velocity.clear();    // Joint velocities (not controlled)
  msg.effort.clear();      // Joint torques/forces (not controlled)
  
  // Publish the message to any listening nodes
  // This sends the joint commands through the ROS2 communication system
  publisher_->publish(msg);
  
  /*
   * TIME UPDATE AND LOGGING
   * =======================
   * Advance the motion timer and provide periodic status updates.
   */
  
  // Update elapsed time for next iteration
  // This advances the sine wave calculation for continuous smooth motion
  time_elapsed_ += 0.02;  // 20ms increment (matches timer period exactly)
  
  // Periodic logging to monitor system status (every 2 seconds)
  // Modulo arithmetic prevents excessive log output while showing progress
  if (static_cast<int>(time_elapsed_ * 10) % 20 == 0) {
    RCLCPP_INFO(this->get_logger(), "Published smooth joint state - Wave position: %.3f", wave_position);
  }
}

} // namespace g1_manipulation

/*
 * MAIN FUNCTION: ROS2 NODE ENTRY POINT
 * ====================================
 * 
 * This is the standard pattern for ROS2 C++ node executables.
 * The main function handles ROS2 initialization, node creation,
 * execution, and cleanup.
 */
int main(int argc, char * argv[])
{
  /*
   * ROS2 INITIALIZATION
   * ==================
   * Initialize the ROS2 C++ client library.
   * Must be called before creating any ROS2 objects.
   */
  rclcpp::init(argc, argv);
  
  /*
   * NODE EXECUTION
   * =============
   * Create the node and start the ROS2 event loop.
   * make_shared creates a shared pointer for automatic memory management.
   * spin() blocks here until the node is shut down (Ctrl+C).
   */
  rclcpp::spin(std::make_shared<g1_manipulation::JointStatePublisher>());
  
  /*
   * CLEANUP
   * =======
   * Shutdown ROS2 and cleanup resources.
   * Called automatically when spin() returns.
   */
  rclcpp::shutdown();
  
  // Return success status to the operating system
  return 0;
} 