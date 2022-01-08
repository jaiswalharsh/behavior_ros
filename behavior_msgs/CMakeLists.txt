cmake_minimum_required(VERSION 3.0.2)
# Project name
project(behavior_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        actionlib_msgs
        geometry_msgs
        std_msgs
)

# add_message_files(
#         DIRECTORY
#                 msg
#         FILES
# )

# add_service_files(
#         DIRECTORY
#                 srv
#         FILES
# )

add_action_files(
        DIRECTORY
                action
        FILES
                TakeOff.action
                EverythingElse.action
)

generate_messages(
        DEPENDENCIES
        actionlib_msgs
        std_msgs
        geometry_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        actionlib_msgs
        geometry_msgs
        std_msgs
)
