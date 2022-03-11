cmake_minimum_required(VERSION 2.8.3)

project(cobot_arm_urdf)

find_package(catkin REQUIRED COMPONENTS roscpp tf)

catkin_package()

find_package(roslaunch)

find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
link_libraries(${kdl_parser_LIBRARIES} ${orocos_kdl_LIBRARIES} /opt/ros/melodic/lib)
include_directories(/opt/ros/melodic/include /usr/include/eigen3)


foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


# Plugin : Kinematics - 6 DOF Manipulator
add_executable(armplugin src/plugin.cpp)
target_link_libraries(armplugin ${catkin_LIBRARIES})

find_package(Boost COMPONENTS system filesystem REQUIRED)

# Node : Teleoperation - Keyboard
add_executable(teleop src/teleop_key.cpp)
target_link_libraries(teleop ${catkin_LIBRARIES})
target_link_libraries(teleop ${Boost_FILESYSTEM_LIBRARY})
target_link_libraries(teleop ${Boost_SYSTEM_LIBRARY})

# Node : Record and Play - Cobot
add_executable(record_and_play src/record_and_play.cpp)
target_link_libraries(record_and_play ${catkin_LIBRARIES})
target_link_libraries(record_and_play ${Boost_FILESYSTEM_LIBRARY})
target_link_libraries(record_and_play ${Boost_SYSTEM_LIBRARY})



# Testing

add_executable(kinematics src/kinematics.cpp)
target_link_libraries(kinematics ${catkin_LIBRARIES} /opt/ros/melodic/lib)

add_executable(iter src/iter_ik.cpp)
target_link_libraries(iter ${catkin_LIBRARIES})

add_executable(trajectory /home/ubuntussd/cpp_ws/dependency/orocos_kinematics_dynamics/orocos_kdl/examples/trajectory_example.cpp)
target_link_libraries(trajectory ${catkin_LIBRARIES})
