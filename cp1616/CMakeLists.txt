cmake_minimum_required(VERSION 2.8.3)
project(cp1616)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  message_generation
  )

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include/)


SET(SOURCES ${PROJECT_SOURCE_DIR}/src/cp1616_io_controller.cpp
            ${PROJECT_SOURCE_DIR}/src/cp1616_io_controller_callbacks.cpp
            ${PROJECT_SOURCE_DIR}/src/cp1616_io_device.cpp
            ${PROJECT_SOURCE_DIR}/src/cp1616_io_device_callbacks.cpp
            ${PROJECT_SOURCE_DIR}/src/cp1616_main.cpp
          )

SET(HEADERS ${PROJECT_SOURCE_DIR}/include/cp1616/cp1616_io_controller.h
            ${PROJECT_SOURCE_DIR}/include/cp1616/cp1616_io_controller_callbacks.h
            ${PROJECT_SOURCE_DIR}/include/cp1616/cp1616_io_device.h
            ${PROJECT_SOURCE_DIR}/include/cp1616/cp1616_io_device_callbacks.h
	  )
	  
catkin_package(
  DEPENDS
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp 
  LIBRARIES
  roscpp
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARIES} ${catkin_LIBRARIES}
                      ${LD_LIBRARY_PATH}libpniousr.a ${LD_LIBRARY_PATH}libpniousr.so
                      ${LD_LIBRARY_PATH}libl2interface.so ${LD_LIBRARY_PATH}libl2interface.a)
