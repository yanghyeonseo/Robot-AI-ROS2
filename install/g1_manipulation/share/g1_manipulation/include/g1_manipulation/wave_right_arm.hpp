/*
 * ROS2 C++ Header File: Right Arm Waving Motion Controller
 * ========================================================
 * 
 * This header file demonstrates:
 * 1. C++ header guard patterns for preventing multiple inclusions
 * 2. ROS2 C++ class design and inheritance
 * 3. Publisher and Timer declarations
 * 4. Member variable organization
 * 5. Namespace usage in ROS2 packages
 * 
 * Educational Topics Covered:
 * - Header guards (#ifndef/#define/#endif)
 * - Forward declarations and includes
 * - Class interface design (public/private separation)
 * - ROS2 C++ node inheritance pattern
 * - Smart pointer usage (SharedPtr)
 * - STL container usage (vector)
 */

#ifndef G1_MANIPULATION__WAVE_RIGHT_ARM_HPP_
#define G1_MANIPULATION__WAVE_RIGHT_ARM_HPP_

/*
 * INCLUDE SECTION: External Dependencies
 * =====================================
 * Order matters for compilation efficiency:
 * 1. System/standard library headers
 * 2. Third-party library headers  
 * 3. ROS2 headers
 * 4. Local project headers
 */

// ROS2 Core Libraries
#include <rclcpp/rclcpp.hpp>                    // Main ROS2 C++ client library
#include <sensor_msgs/msg/joint_state.hpp>     // JointState message definition

// Standard C++ Libraries
#include <cmath>        // Mathematical functions (sin, cos, M_PI)
#include <vector>       // Dynamic arrays for joint data
#include <string>       // String class for joint names

/*
 * NAMESPACE: Package Organization
 * ==============================
 * Namespaces prevent naming conflicts and organize code logically.
 * ROS2 convention: use package name as namespace
 */
namespace g1_manipulation
{

/*
 * CLASS DECLARATION: JointStatePublisher
 * =====================================
 * 
 * This class inherits from rclcpp::Node to create a ROS2 node that:
 * - Publishes JointState messages for robot control
 * - Uses a timer for periodic execution
 * - Generates smooth waving motion for the right arm
 * 
 * Design Pattern: Composition over inheritance
 * - Node functionality through inheritance
 * - Specific behavior through member functions
 */
class JointStatePublisher : public rclcpp::Node
{
public:
  /*
   * PUBLIC INTERFACE
   * ===============
   * Only the constructor is public - this is the minimal interface
   * needed to create and use the node. All implementation details
   * are hidden in the private section.
   */
  
  /**
   * Constructor: Initialize the ROS2 node
   * 
   * Sets up publishers, timers, and motion parameters.
   * Called once when the node is created.
   */
  JointStatePublisher();

private:
  /*
   * PRIVATE IMPLEMENTATION
   * =====================
   * Member functions and variables that are internal to the class.
   * This separation provides encapsulation - users of the class
   * don't need to know about these implementation details.
   */
  
  /**
   * Timer Callback Function
   * 
   * Called periodically (50Hz) to publish joint state messages.
   * Generates smooth waving motion using mathematical functions.
   */
  void publish_joint_states();
  
  /*
   * MEMBER VARIABLES: Node State and Configuration
   * =============================================
   * These variables store the node's state and configuration.
   * Using SharedPtr for ROS2 objects enables proper memory management.
   */
  
  // ROS2 Communication Objects
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;  // Publishes joint commands
  rclcpp::TimerBase::SharedPtr timer_;                                    // Triggers periodic execution
  
  // Motion Control Variables
  double time_elapsed_;    // Accumulated time for sine wave calculations (seconds)
  double wave_period_;     // Complete wave cycle duration (seconds)
  
  // Robot Configuration Data
  std::vector<std::string> joint_names_;     // Names of all robot joints (must match URDF)
  std::vector<double> base_positions_;       // Neutral positions for all joints (radians)
};

} // namespace g1_manipulation

/*
 * HEADER GUARD CLOSING
 * ===================
 * This closes the header guard that prevents multiple inclusions.
 * The macro name should match the opening #ifndef exactly.
 */
#endif // G1_MANIPULATION__WAVE_RIGHT_ARM_HPP_ 