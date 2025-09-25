# =============================================================================
# ROS2 Package CMakeLists.txt: Mixed Python/C++ Package Configuration
# =============================================================================
#
# This CMakeLists.txt demonstrates:
# 1. ROS2 package build system configuration
# 2. Mixed Python and C++ development setup
# 3. Dependency management and linking
# 4. Resource installation and packaging
# 5. Testing framework integration
#
# Educational Topics Covered:
# - CMake basics and ROS2 extensions
# - Package dependency declaration
# - C++ compilation and linking
# - Python package integration
# - Resource file installation
# - Executable creation and installation
# - Testing and linting configuration
#
# Build Commands:
# - Full build: colcon build --packages-select g1_manipulation
# - Clean build: colcon build --packages-select g1_manipulation --cmake-clean-cache
# - Test: colcon test --packages-select g1_manipulation
# =============================================================================

# CMake Version Requirement
# Specifies minimum CMake version needed for this package
# 3.8+ required for modern CMake features used by ROS2
cmake_minimum_required(VERSION 3.8)

# Project Declaration
# Defines the package name - must match package.xml <name> tag
project(g1_manipulation)

# =============================================================================
# COMPILER CONFIGURATION
# =============================================================================

# Enable additional compiler warnings for better code quality
# These flags help catch potential issues during compilation
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================================================================
# DEPENDENCY DISCOVERY
# =============================================================================
# Find and configure all required ROS2 packages and external dependencies
# Each find_package() call locates libraries and sets up CMake variables

# Core ROS2 Build System Dependencies
find_package(ament_cmake REQUIRED)           # Main ROS2 CMake macros and functions
find_package(ament_cmake_python REQUIRED)    # Python integration support

# Language-Specific ROS2 Client Libraries
find_package(rclpy REQUIRED)                 # ROS2 Python client library
find_package(rclcpp REQUIRED)                # ROS2 C++ client library

# ROS2 Message and Interface Packages
find_package(sensor_msgs REQUIRED)           # Standard sensor message definitions

# =============================================================================
# RESOURCE FILE INSTALLATION
# =============================================================================
# Install package resources (launch files, config files, etc.) to share directory
# These files are accessible to other packages and launch systems

install(
  DIRECTORY
    launch                    # Launch files for starting nodes and systems
    urdf                      # Robot description files (URDF/Xacro)
    meshes                    # 3D model files for visualization
    rviz                      # RViz configuration files
    include                   # C++ header files (public interface)
    src                       # C++ source files (for reference/debugging)
    g1_manipulation           # Python package directory
  DESTINATION share/${PROJECT_NAME}  # Install to: install/share/g1_manipulation/
)

# =============================================================================
# PYTHON PACKAGE INTEGRATION
# =============================================================================
# Configure Python components of the mixed-language package

# Install Python package using standard Python packaging
# This makes the g1_manipulation Python module importable
ament_python_install_package(${PROJECT_NAME})

# Install Python executables (scripts that can be run with ros2 run)
install(
  PROGRAMS
    g1_manipulation/wave_left_arm.py    # Python node for left arm waving
  DESTINATION lib/${PROJECT_NAME}       # Install to: install/lib/g1_manipulation/
)

# =============================================================================
# C++ EXECUTABLE CONFIGURATION
# =============================================================================
# Configure C++ components: compilation, linking, and installation

# Include Directories Setup
# Tell the compiler where to find header files
include_directories(include)            # Local header files in include/ directory

# C++ Executable Definition
# Create executable from source files
add_executable(wave_right_arm src/wave_right_arm.cpp)

# Dependency Linking for C++ Executable
# Link the executable against required ROS2 libraries
# ament_target_dependencies automatically handles include paths and linking
ament_target_dependencies(wave_right_arm 
  rclcpp                   # ROS2 C++ client library
  sensor_msgs              # JointState message definitions
)

# C++ Executable Installation
# Install compiled executables so they can be run with ros2 run
install(TARGETS
  wave_right_arm            # Executable name
  DESTINATION lib/${PROJECT_NAME}  # Install to: install/lib/g1_manipulation/
)

# =============================================================================
# TESTING AND CODE QUALITY
# =============================================================================
# Configure testing framework and code quality checks

if(BUILD_TESTING)
  # Find testing framework
  find_package(ament_lint_auto REQUIRED)
  
  # Skip copyright checking (commented lines explain when to enable)
  # Uncomment next line when copyright headers are added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  
  # Skip C++ linting (commented lines explain when to enable)  
  # Uncomment next line when package is in git repo with proper copyright headers
  set(ament_cmake_cpplint_FOUND TRUE)
  
  # Run automatic linting and testing
  # This finds and runs all available linters for the package
  ament_lint_auto_find_test_dependencies()
endif()

# =============================================================================
# PACKAGE FINALIZATION
# =============================================================================
# Complete the package configuration and generate necessary files

# Generate package configuration files
# This creates the files needed for other packages to find and use this package
ament_package()
