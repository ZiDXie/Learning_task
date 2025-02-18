cmake_minimum_required(VERSION 3.10)
project(hw)

find_package(catkin REQUIRED
        COMPONENTS
)
###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS
)
include_directories(
        #        include
        ${catkin_INCLUDE_DIRS}
)
