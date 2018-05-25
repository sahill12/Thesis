cmake_minimum_required (VERSION 2.8.12)
project (attention_sfa_robot_control)

add_compile_options ("-Wall" "-std=c++11")

list (APPEND CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH})
include (CommonMacros)

require_custom_package (OpenCV 2)
include_directories (${OpenCV_INCLUDE_DIRS})

find_package (catkin REQUIRED COMPONENTS
	dynamic_reconfigure
	geometry_msgs
	message_filters
	message_generation
	nav_msgs
	roscpp
	sensor_msgs
	std_msgs
	tf
	attention_signal
)

generate_dynamic_reconfigure_options (
	cfg/Control_parameter.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES atir
	CATKIN_DEPENDS
		dynamic_reconfigure
		geometry_msgs
		message_filters
		nav_msgs
		roscpp
		sensor_msgs
		std_msgs
		tf
		attention_signal
		
	#  DEPENDS system_lib
)


include_directories (
  ${catkin_INCLUDE_DIRS}
)


add_executable (robot_control src/robot_control.cpp)
target_link_libraries (robot_control
   ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)
    


add_dependencies (robot_control ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS})

