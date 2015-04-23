cmake_minimum_required(VERSION 2.8.3)
project(nao_mocap)

set(CMAKE_CXX_FLAGS " -std=gnu++0x ${CMAKE_CXX_FLAGS}")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  naoqi_msgs
  roscpp
  tf
  actionlib
  std_srvs
  move_base_msgs
  nav_msgs
)

find_package(NAOqi QUIET COMPONENTS
    alcommon
    alvision
    alproxies
    alerror
    alvalue
)

find_package(Boost QUIET COMPONENTS)

if( ${NAOqi_FOUND} AND ${Boost_FOUND} )
  message(STATUS "NAOqi C++ SDK found, Proceed with building")

    catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS geometry_msgs roscpp tf naoqi_msgs std_srvs actionlib move_base_msgs nav_msgs
    )

    include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${NAOqi_INCLUDE_DIRS}
    )

    add_executable(nao_mocap_node src/nao_mocap_node.cpp)
    target_link_libraries(nao_mocap_node     
       ${catkin_LIBRARIES}
       ${NAOqi_LIBRARIES}
       ${Boost_LIBRARIES}
    )

    add_dependencies(nao_mocap_node ${catkin_EXPORTED_TARGETS})

    install(TARGETS nao_mocap_node
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
else()
  message(FATAL_ERROR "Could not find NAOqi C++ SDK; Aborting catkin")
endif()