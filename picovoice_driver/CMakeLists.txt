cmake_minimum_required(VERSION 3.0.2)
project(picovoice_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  picovoice_msgs
  roscpp
  roslib
)

find_package(ALSA REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    roslib
)

###########
## Build ##
###########

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/)
find_package(PORTAUDIO REQUIRED)

set(PICOVOICE_DIR extern/picovoice)
set(PICOVOICE_INCLUDE_DIRS ${PICOVOICE_DIR}/include)
set(PICOVOICE_LIBRARIES ${PROJECT_SOURCE_DIR}/${PICOVOICE_DIR}/lib/libpv_rhino.so)

include_directories(
  ${ALSA_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PICOVOICE_INCLUDE_DIRS}
  ${PORTAUDIO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/recognizer.cpp
  src/rhino_recognizer.cpp
  src/util.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${ALSA_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PICOVOICE_LIBRARIES}
  ${PORTAUDIO_LIBRARIES}
)

add_executable(${PROJECT_NAME}_rhino src/rhino_node.cpp)
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
    ${PROJECT_NAME}_rhino
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
