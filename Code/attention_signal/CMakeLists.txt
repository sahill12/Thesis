cmake_minimum_required(VERSION 2.8.12)

project(attention_signal)

find_package(catkin REQUIRED
	COMPONENTS
		std_msgs
		message_generation
		geometry_msgs
)

add_message_files (FILES
    Signal.msg
	SignalArray.msg

)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
    #  INCLUDE_DIRS include
	#  LIBRARIES test
	
	CATKIN_DEPENDS
	    message_runtime
		std_msgs
		geometry_msgs
		#  DEPENDS system_lib
)
