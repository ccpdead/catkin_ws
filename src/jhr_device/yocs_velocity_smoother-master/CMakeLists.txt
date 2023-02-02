cmake_minimum_required(VERSION 2.8.3)
project(yocs_velocity_smoother)
find_package(catkin REQUIRED COMPONENTS 

roscpp 
pluginlib 
nodelet 
ecl_threads 
dynamic_reconfigure 
geometry_msgs 
nav_msgs

)

# 动态参数表位置。
generate_dynamic_reconfigure_options(
        cfg/params.cfg
        #....
        )

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}_nodelet
   CATKIN_DEPENDS 

   roscpp 
   pluginlib 
   nodelet 
   ecl_threads 
   dynamic_reconfigure 
   geometry_msgs 
   nav_msgs

)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include ${catkin_INCLUDE_DIRS})
#编译成库模式。
add_library(${PROJECT_NAME}_nodelet src/velocity_smoother_nodelet.cpp)

#链接动态链接库
target_link_libraries(${PROJECT_NAME}_nodelet ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_gencfg)


install(TARGETS ${PROJECT_NAME}_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
