cmake_minimum_required(VERSION 3.0.2)
project(picovoice_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  ddynamic_reconfigure
  picovoice_msgs
  roscpp
  roslib
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    ddynamic_reconfigure
    roscpp
    roslib
)

###########
## Build ##
###########

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/)
find_package(PORTAUDIO REQUIRED)
find_package(LIBSNDFILE REQUIRED)

set(PICOVOICE_DIR extern/picovoice)
set(PICOVOICE_INCLUDE_DIRS ${PICOVOICE_DIR}/include)
set(PICOVOICE_LIBRARIES
  ${PROJECT_SOURCE_DIR}/${PICOVOICE_DIR}/lib/${CMAKE_HOST_SYSTEM_PROCESSOR}/libpv_porcupine.so
  ${PROJECT_SOURCE_DIR}/${PICOVOICE_DIR}/lib/${CMAKE_HOST_SYSTEM_PROCESSOR}/libpv_rhino.so
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${LIBSNDFILE_INCLUDE_DIRS}
  ${PICOVOICE_INCLUDE_DIRS}
  ${PORTAUDIO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/porcupine_recognizer.cpp
  src/recognizer.cpp
  src/rhino_recognizer.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${LIBSNDFILE_LIBRARIES}
  ${PICOVOICE_LIBRARIES}
  ${PORTAUDIO_LIBRARIES}
)

add_executable(${PROJECT_NAME}_porcupine
  src/porcupine_node.cpp
  src/ros_util.cpp
)
target_link_libraries(${PROJECT_NAME}_porcupine ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_rhino
  src/rhino_node.cpp
  src/ros_util.cpp
)
target_link_libraries(${PROJECT_NAME}_rhino ${PROJECT_NAME})

#############
## Install ##
#############

install(FILES ${PICOVOICE_LIBRARIES}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY ${PICOVOICE_INCLUDE_DIRS}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ${PICOVOICE_DIR}/resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${PICOVOICE_DIR}
)

install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_porcupine
    ${PROJECT_NAME}_rhino
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
