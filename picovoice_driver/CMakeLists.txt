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

############
### Build ##
############

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/)
find_package(LIBSNDFILE REQUIRED)

set(EXTERN ${PROJECT_SOURCE_DIR}/${PICOVOICE_DIR}/extern)
set(PICOVOICE_INCLUDE_DIRS
  ${EXTERN}/porcupine/include
  ${EXTERN}/rhino/include
  ${EXTERN}/pvrecorder/include
)
set(PICOVOICE_LIBRARIES
  ${EXTERN}/porcupine/lib/linux/${CMAKE_SYSTEM_PROCESSOR}/libpv_porcupine.so
  ${EXTERN}/rhino/lib/linux/${CMAKE_SYSTEM_PROCESSOR}/libpv_rhino.so
  ${EXTERN}/pvrecorder/lib/linux/${CMAKE_SYSTEM_PROCESSOR}/libpv_recorder.so
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${LIBSNDFILE_INCLUDE_DIRS}
  ${PICOVOICE_INCLUDE_DIRS}
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
)

add_executable(test_${PROJECT_NAME}_porcupine
  test/test_porcupine.cpp
)
target_link_libraries(test_${PROJECT_NAME}_porcupine ${PROJECT_NAME})

add_executable(test_${PROJECT_NAME}_rhino
  test/test_rhino.cpp
)
target_link_libraries(test_${PROJECT_NAME}_rhino ${PROJECT_NAME})

#add_executable(${PROJECT_NAME}_porcupine
#  src/porcupine_node.cpp
#  src/ros_util.cpp
#)
#target_link_libraries(${PROJECT_NAME}_porcupine ${PROJECT_NAME})

#add_executable(${PROJECT_NAME}_rhino
#  src/rhino_node.cpp
#  src/ros_util.cpp
#)
#target_link_libraries(${PROJECT_NAME}_rhino ${PROJECT_NAME})

##############
### Install ##
##############

#install(FILES ${PICOVOICE_LIBRARIES}
#  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

#install(DIRECTORY ${PICOVOICE_INCLUDE_DIRS}
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)

#install(DIRECTORY ${PICOVOICE_DIR}/resources
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${PICOVOICE_DIR}
#)

#install(
#  TARGETS
#    ${PROJECT_NAME}
#    ${PROJECT_NAME}_porcupine
#    ${PROJECT_NAME}_rhino
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)
