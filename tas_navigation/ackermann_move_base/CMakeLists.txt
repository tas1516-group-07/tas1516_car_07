cmake_minimum_required(VERSION 2.8.3)
project(ackermann_move_base)

find_package(catkin REQUIRED
    COMPONENTS
        cmake_modules
        roscpp
        pluginlib
        actionlib
        dynamic_reconfigure
        message_generation
        nav_core
        tf
)
find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/AckermannMoveBase.cfg
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        dynamic_reconfigure
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

# ackermann_move_base
add_library(ackermann_move_base
  src/ackermann_move_base.cpp
)
target_link_libraries(ackermann_move_base
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_dependencies(ackermann_move_base ackermann_move_base_gencfg)
add_dependencies(ackermann_move_base geometry_msgs_gencpp)

add_executable(ackermann_move_base_node
  src/ackermann_move_base_node.cpp
)
target_link_libraries(ackermann_move_base_node ackermann_move_base)
set_target_properties(ackermann_move_base_node PROPERTIES OUTPUT_NAME ackermann_move_base)

install(
    TARGETS
        ackermann_move_base
        ackermann_move_base_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
