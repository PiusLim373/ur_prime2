cmake_minimum_required(VERSION 3.8)
project(prime2_master)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(prime2_master REQUIRED)
find_package(tf2 REQUIRED)
    


set(srv_files
  "srv/StringTrigger.srv"
  "srv/Move.srv"
  "srv/VisualServo.srv"
  "srv/ForceTorqueServo.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(mgi_node src/mgi_node.cpp src/mgi.cpp)

target_include_directories(mgi_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mgi_node rclcpp moveit_ros_planning_interface std_srvs std_msgs sensor_msgs geometry_msgs tf2_ros tf2_geometry_msgs tf2)
rosidl_target_interfaces(mgi_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS mgi_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

file(GLOB SCRIPT_FILES scripts/*.py ) 
install(PROGRAMS
  ${SCRIPT_FILES}
  DESTINATION lib/${PROJECT_NAME} 
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
