cmake_minimum_required(VERSION 3.0.2)
project(quarotor_feedback_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  nav_msgs
  rospy
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES Quarotor_Feedback_Controller
#  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs
#  DEPENDS system_lib
)

# catkin_install_python(PROGRAMS scripts/NetworkPlanner
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#             src/QuarotorFeedbackController.cpp
# )

add_executable(quarotor_feedback_controller0 
              src/OffboardNode0.cpp
              src/OffboardWrapper.cpp
              src/QuadrotorFeedbackController.cpp
              src/QuadrotorAggressiveController.cpp
              src/ParamLoad.cpp
)

add_executable(quarotor_feedback_controller1 
              src/OffboardNode1.cpp
              src/OffboardWrapper.cpp
              src/QuadrotorFeedbackController.cpp
              src/QuadrotorAggressiveController.cpp
              src/ParamLoad.cpp
)

target_link_libraries(quarotor_feedback_controller0
  ${catkin_LIBRARIES}
)

target_link_libraries(quarotor_feedback_controller1
  ${catkin_LIBRARIES}
)
