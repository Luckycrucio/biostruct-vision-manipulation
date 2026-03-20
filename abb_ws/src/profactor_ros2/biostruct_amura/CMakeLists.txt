cmake_minimum_required(VERSION 3.8)
project(biostruct_amura)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find all dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)

# PHASE ONE: 
add_executable(planning_node src/planning_node.cpp)
target_include_directories(planning_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_definitions(planning_node PRIVATE PROJECT_DIR="${CMAKE_SOURCE_DIR}")
target_compile_features(planning_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++1
ament_target_dependencies(
  planning_node
  "moveit_visual_tools"
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_core"
  "rclcpp_action"
  "moveit_msgs"
  "geometry_msgs"
  "rosbag2_cpp"
  "control_msgs"
  "moveit_ros_planning"
  "geometric_shapes"
  "ament_index_cpp"
)
target_link_libraries(planning_node
  ${OpenCV_LIBS}
)
install(TARGETS planning_node
  DESTINATION lib/${PROJECT_NAME}
)

# PHASE TWO: 
add_executable(execution_node src/execution_node.cpp)
target_include_directories(execution_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(execution_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  execution_node
  "moveit_visual_tools"
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_core"
  "rclcpp_action"
  "moveit_msgs"
  "geometry_msgs"
  "rosbag2_cpp"
  "control_msgs"
  "moveit_ros_planning"
  "geometric_shapes"
)
install(TARGETS execution_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
