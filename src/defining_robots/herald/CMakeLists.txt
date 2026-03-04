cmake_minimum_required(VERSION 3.5)
project(herald)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# find_package(roslaunch)

install(DIRECTORY
	config 
	launch 
	meshes 
	urdf
	DESTINATION share/${PROJECT_NAME}/
)

ament_package()