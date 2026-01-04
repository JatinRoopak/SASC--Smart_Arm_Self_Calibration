cmake_minimum_required(VERSION 3.8)
project(sasc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)

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

install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

add_executable(camera_reader src/camera_reader.cpp)
add_executable(arm_commander src/arm_commander.cpp)


ament_target_dependencies(camera_reader
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)
ament_target_dependencies(arm_commander
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
  moveit_core
)

target_link_libraries(camera_reader ${OpenCV_LIBRARIES})

install(TARGETS camera_reader DESTINATION lib/${PROJECT_NAME})
install(TARGETS arm_commander DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch_files/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
