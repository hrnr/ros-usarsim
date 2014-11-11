cmake_minimum_required(VERSION 2.8.3)
project(usarsim_inf)

# build dependencies
find_package(catkin REQUIRED COMPONENTS roscpp tf actionlib message_generation 
	std_msgs geometry_msgs nav_msgs sensor_msgs control_msgs)
find_package(Boost REQUIRED COMPONENTS thread system)
# include_directories(${Boost_INCLUDE_DIRS})
# include_directories(${catkin_INCLUDE_DIRS})


# generate messages
add_message_files(FILES
	EffectorCommand.msg
	RangeImageScan.msg
	ToolchangerStatus.msg
	EffectorStatus.msg
	SenseObject.msg
	ToolType.msg)
generate_messages(DEPENDENCIES 
	std_msgs geometry_msgs nav_msgs sensor_msgs control_msgs)

# runtime dependencies
catkin_package(
	CATKIN_DEPENDS roscpp tf actionlib message_runtime
		std_msgs geometry_msgs nav_msgs sensor_msgs control_msgs
	LIBRARIES ${PROJECT_NAME})

# build targets
set(usarsim_inf_SOURCES
	src/ulapi.cpp
	src/genericInf.cpp
	src/servoInf.cpp
	src/usarsimInf.cpp
	src/usarsimMisc.cpp
	src/simware.cpp)

add_library(usarsim_inf ${usarsim_inf_SOURCES})
add_executable(usarsim_node src/usarsim.cpp)
add_executable(usarsim_urdf src/usarsim_urdf_gen.cpp)

add_dependencies(usarsim_inf ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(usarsim_node usarsim_inf ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(usarsim_urdf usarsim_inf ${Boost_LIBRARIES} ${catkin_LIBRARIES})

# install roslaunch files
install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)