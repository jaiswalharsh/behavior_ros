cmake_minimum_required(VERSION 3.0.2)
### Project name
project(behavior_execution)

## Use C++14
set(CMAKE_CXX_STANDARD 14)



### Find dependencies
find_package(catkin REQUIRED COMPONENTS # Must be same as CATKIN_DEPENDS
        message_generation
        roscpp
        tf
        geometry_msgs
        std_msgs
        behavior_msgs
        inspectrone_library
)

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)



############
## catkin ##
############

### Define package
catkin_package(
        INCLUDE_DIRS 
                include
                ${EIGEN3_INCLUDE_DIR} # This is only necessary because Eigen3 sets a non-standard EIGEN3_INCLUDE_DIR variable
        LIBRARIES 
                inspection_execution
        CATKIN_DEPENDS # Must be same as CATKIN_DEPENDS
        roscpp
        tf
        geometry_msgs
        std_msgs
        behavior_msgs
        inspectrone_library
)



###########
## Build ##
###########


# Specify additional locations of header files
include_directories(
        include
        ${THIS_PACKAGE_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR} # Set manually because Eigen sets a non standard INCLUDE DIR
        ${Boost_INCLUDE_DIRS} # Set because Boost is an internal dependency, not transitive.
)

# Declare a cpp library
add_library(inspection_execution 
        src/inspection_execution.cpp
)
target_compile_features(inspection_execution INTERFACE cxx_std_11)
target_link_libraries(inspection_execution
        ${catkin_LIBRARIES}
)

# Declare cpp executables
add_executable(inspection_execution_node 
        src/inspection_execution_node.cpp
)
target_compile_features(inspection_execution_node INTERFACE cxx_std_11)
target_link_libraries(inspection_execution_node
        inspection_execution
        ${catkin_LIBRARIES}
)



#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
        TARGETS inspection_execution_node inspection_execution
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
# install(
#         FILES launch/inspection_fiducials.launch launch/explore_and_inspect.launch
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )
