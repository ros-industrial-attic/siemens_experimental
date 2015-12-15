cmake_minimum_required(VERSION 2.8.3)
project(siemens_cp1616)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  message_generation
  )

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include/)

ADD_LIBRARY(siemens_cp1616
	${PROJECT_SOURCE_DIR}/src/siemens_cp1616_io_controller.cpp
        ${PROJECT_SOURCE_DIR}/src/siemens_cp1616_io_controller_callbacks.cpp
        ${PROJECT_SOURCE_DIR}/src/siemens_cp1616_io_device.cpp
        ${PROJECT_SOURCE_DIR}/src/siemens_cp1616_io_device_callbacks.cpp
        )

add_service_files(FILES set_alarm.srv
                        reset_alarm.srv
                 )

generate_messages(DEPENDENCIES)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES siemens_cp1616
  CATKIN_DEPENDS roscpp 
  DEPENDS libpnoiusr libl2interface 
)
#Link CP1616 Library with yaml-cpp
target_link_libraries (siemens_cp1616 yaml-cpp)

#IO Controller wrapper
add_executable(siemens_cp1616_io_controller_wrapper ${PROJECT_SOURCE_DIR}/src/siemens_cp1616_io_controller_wrapper.cpp)
target_link_libraries(siemens_cp1616_io_controller_wrapper ${catkin_LIBRARIES} 
                                                           siemens_cp1616
                                                           libpniousr.a
                                                           libpniousr.so
                                                           libl2interface.so
                                                           libl2interface.a
                                                           pthread
                                                           )
                                                           

#IO Device wrapper                                                   
add_executable(siemens_cp1616_io_device_wrapper ${PROJECT_SOURCE_DIR}/src/siemens_cp1616_io_device_wrapper.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} siemens_cp1616_io_device_wrapper_gencpp)
target_link_libraries(siemens_cp1616_io_device_wrapper ${catkin_LIBRARIES} 
                                                       siemens_cp1616 
                                                       libpniousr.a
                                                       libpniousr.so
                                                       libl2interface.so
                                                       libl2interface.a
                                                       pthread
                                                       )


## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_io_controller_wrapper ${PROJECT_NAME}_io_device_wrapper
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)                                       