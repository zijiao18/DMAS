cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
cmake_policy(SET CMP0042 NEW)

project(distributed_multi_agent_exploration)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(bot_controller SHARED bot/bot_controller.cpp utilities/flag/flag.h utilities/occupancy_grid/occupancy_grid.cpp utilities/block_q/block_q.h utilities/logger/logger.cpp)
target_link_libraries(bot_controller
  ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES}
)

add_library(laser_sensor SHARED sensor/laser_sensor.cpp utilities/flag/flag.h)
target_link_libraries(laser_sensor
  ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES}
)

add_library(bot_factory SHARED factory/bot_factory.cpp utilities/block_q/block_q.h utilities/logger/logger.cpp)
target_link_libraries(bot_factory
  ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES}
)
