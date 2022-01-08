cmake_minimum_required(VERSION 3.0.2)
### Project name
project(behavior_tree)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



### Find dependencies
find_package(catkin REQUIRED COMPONENTS # Must be same as CATKIN_DEPENDS
        roscpp
        actionlib
        behaviortree_cpp_v3
        diagnostic_msgs
        geometry_msgs
        behavior_msgs
        # inspectrone_library
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
                ${PROJECT_NAME}
        CATKIN_DEPENDS # Must be same as CATKIN_DEPENDS
                roscpp
                actionlib
                behaviortree_cpp_v3
                diagnostic_msgs
                geometry_msgs
                behavior_msgs
                # inspectrone_library
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
add_library(behavior_tree
        src/inspection_behavior.cpp
)
target_compile_features(behavior_tree INTERFACE cxx_std_11)
target_link_libraries(behavior_tree
        ${catkin_LIBRARIES}
)

add_library(inspection_nodes 
        src/inspection_nodes.cpp
)
target_compile_features(inspection_nodes INTERFACE cxx_std_11)
target_link_libraries(inspection_nodes
        ${catkin_LIBRARIES}
)


# Declare cpp executables
add_executable(inspection_behavior_node 
        src/inspection_behavior_node.cpp
)
target_compile_features(inspection_behavior_node INTERFACE cxx_std_11)
target_link_libraries(inspection_behavior_node
        behavior_tree
        inspection_nodes        
        ${catkin_LIBRARIES}
)



#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
        TARGETS inspection_behavior_node behavior_tree
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
#         FILES launch/some_launch.launch
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )
