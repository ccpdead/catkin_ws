cmake_minimum_required(VERSION 3.14.5)
project(tof_dev_sdk_demo)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp rospy 
  std_msgs 
  genmsg 
  message_generation
  sensor_msgs 
  cv_bridge
  # pcl_conversions

)

find_package(PCL REQUIRED
PATHS ~/pcl-19/install
NO_DEFAULT_PATH)

find_package(OpenCV REQUIRED)


## Declare ROS messages and services
#add_message_files(FILES XXX.msg)
#add_service_files(FILES XXX.srv)


## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)


## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)


## Build publisher and subscribe


message(STATUS " CMAKE_SOURCE_DIR=========== ${CMAKE_SOURCE_DIR}")


set(CMAKE_BUILD_TYPE Release)

set(DEMO_PROJECT_DIR ${CMAKE_SOURCE_DIR}/dev-ros)
set(DEMO_OUTPUT_DIR ${CMAKE_SOURCE_DIR}/../devel/lib/tof_dev_sdk_demo)

set(DEMO_INCLUDE_DIR ${DEMO_PROJECT_DIR}/include)
set(DEMO_LIBS_DIR ${DEMO_PROJECT_DIR}/libs)
set(DEMO_LIBS_NAME tof_dev_sdk pthread)
set(DEMO_PARAM_DIR ${DEMO_PROJECT_DIR}/parameter)

## Compile as C++11, supported in ROS Melodic and newer
add_compile_options(-std=c++11)
add_definitions(-DLINUX)



# for  -----------------------publisher_demo-----------------------------------
include_directories(${DEMO_INCLUDE_DIR}
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#message(STATUS " DEMO_LIBS_DIR=========== ${DEMO_LIBS_DIR}")

link_directories(${DEMO_LIBS_DIR})
add_executable(publisher_demo demo/tof_dev_sdk_demo-ros-publisher.cpp)
target_link_libraries(publisher_demo ${DEMO_LIBS_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

#add_dependencies(talker XXXX_generate_messages_cpp)



install(DIRECTORY "${DEMO_LIBS_DIR}/" DESTINATION "${DEMO_OUTPUT_DIR}" FILES_MATCHING PATTERN "*" PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ)
install(DIRECTORY "${DEMO_PARAM_DIR}" DESTINATION "${DEMO_OUTPUT_DIR}")

