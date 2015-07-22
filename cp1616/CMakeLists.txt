cmake_minimum_required(VERSION 2.8.3)
project(cp1616)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  )

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include/)

ADD_LIBRARY(cp1616
	${PROJECT_SOURCE_DIR}/src/cp1616_io_controller.cpp
        ${PROJECT_SOURCE_DIR}/src/cp1616_io_controller_callbacks.cpp
        ${PROJECT_SOURCE_DIR}/src/cp1616_io_device.cpp
        ${PROJECT_SOURCE_DIR}/src/cp1616_io_device_callbacks.cpp
        )
  

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cp1616
  CATKIN_DEPENDS roscpp 
  DEPENDS libpnoiusr libl2interface 
)

add_executable(cp1616_test ${PROJECT_SOURCE_DIR}/src/cp1616_main.cpp)
target_link_libraries (cp1616 yaml-cpp)	
target_link_libraries(cp1616_test ${catkin_LIBRARIES} 
                                  cp1616
                                  libpniousr.a
                                  libpniousr.so
                                  libl2interface.so 
                                  libl2interface.a)
