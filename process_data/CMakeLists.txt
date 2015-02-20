cmake_minimum_required(VERSION 2.8.3)
project(process_data)


find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs qt_build image_geometry tf semantic_map metaroom_xml_parser)

set(CMAKE_CXX_FLAGS "-O4 -g -fPIC -std=c++0x -fpermissive")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rosbuild_prepare_qt4(QtCore QtXml)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES metaroom_xml_parser semantic_map
   DEPENDS qt_build image_geometry libqt4-dev
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(process_data src/process_data.cpp )

 target_link_libraries(process_data
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
 )


