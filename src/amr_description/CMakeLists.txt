cmake_minimum_required(VERSION 2.8.3)
project(amr_description)

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   std_msgs
#   sensor_msgs
# )

# catkin_package(

# )


# include_directories(
# # include
#   ${catkin_INCLUDE_DIRS}
# )

# install(DIRECTORY launch/
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#     FILES_MATCHING PATTERN "*.launch"
# )

# install(DIRECTORY meshes/
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
#     FILES_MATCHING PATTERN "*"
# )

# install(DIRECTORY urdf/
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
#     FILES_MATCHING PATTERN "*"
# )

# install(DIRECTORY rviz/
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
#     FILES_MATCHING PATTERN "*"
# )

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

foreach(dir rviz launch meshes urdf config)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
