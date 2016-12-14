cmake_minimum_required(VERSION 2.8.3)

project(homer_robot_face)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	roslib
	std_msgs
	sensor_msgs
	message_generation
	genmsg
	image_transport
	cv_bridge
	cmake_modules
)

#set the default path for built executables to the "bin" directory
#INSTALL(CODE "FILE(MAKE_DIRECTORY \${ENV}\${CMAKE_INSTALL_PREFIX}${mydir})")
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
# Use OGRE from ROS
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/share/OGRE/cmake/modules)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem ${ogre_PACKAGE_PATH}/ogre/include/OGRE")
# QT related
set(QT_USE_QTXML TRUE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")


#for face output
find_package(OGRE REQUIRED COMPONENTS RenderSystem_GL)
add_definitions( -DOGRE_PLUGIN_PATH="/usr/lib/x86_64-linux-gnu/OGRE-1.9.0")
#for GUI visualization
find_package(Qt5 REQUIRED COMPONENTS Core Widgets X11Extras)
#for speak out
find_package(Festival REQUIRED)
#festival depends on that
find_package(EST REQUIRED)
#for resizing image
find_package(OpenCV REQUIRED)

add_message_files(
	FILES
		DisplayImage.msg
		DisplayImageFile.msg
)

generate_messages(
	DEPENDENCIES
		sensor_msgs
)

###########
## Build ##
###########

#set build type
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)	

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#	message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()
include_directories(
	include
	SYSTEM
	${catkin_INCLUDE_DIRS}
	${Qt5Core_INCLUDE_DIRS}
	${Qt5Widgets_INCLUDE_DIRS}
	${Qt5X11Extras_INCLUDE_DIRS}
	${Festival_INCLUDE_DIR}
	${EST_INCLUDE_DIR}
	${OGRE_INCLUDE_DIR}
	#/usr/include/OGRE/
)
#include(${QT_USE_FILE}) 

catkin_package(
	CATKIN_DEPENDS roscpp message_runtime std_msgs sensor_msgs
)

## Declare source files
set(RobotFace_SRC
	src/main.cpp
	src/MainWindow.cpp
	src/TalkingHead.cpp
	src/TextOutDisplay.cpp
	src/QtRosNode.cpp
	src/FestivalGenerator.cpp
	src/RotationLabel.cpp
	src/TextProcessor.cpp
	src/ImageDisplay.cpp
)

# Declare header files (needed to specify all include files for wrapping with QT5)
set(RobotFace_INC
	include/homer_robot_face/MainWindow.h
	include/homer_robot_face/TalkingHead.h
	include/homer_robot_face/TextOutDisplay.h
	include/homer_robot_face/QtRosNode.h
	include/homer_robot_face/FestivalGenerator.h
	include/homer_robot_face/RotationLabel.h
	include/homer_robot_face/ImageDisplay.h
)

QT5_WRAP_CPP(RobotFace_MOC_SRC ${RobotFace_INC})

add_executable(FestivalSynthesizer src/FestivalSynthesizer.cpp src/TextProcessor.cpp )
add_executable(RobotFace ${RobotFace_SRC} ${RobotFace_MOC_SRC})

# fix archlinux build
list(APPEND Festival_LIBRARIES curses)



add_dependencies(RobotFace ${PROJECT_NAME}_gencpp)

target_link_libraries(FestivalSynthesizer
	${catkin_LIBRARIES}
	${Festival_LIBRARIES}
	${EST_LIBRARIES}
	pulse-simple
)

target_link_libraries(RobotFace
	${catkin_LIBRARIES}
	${Qt5Core_LIBRARIES}
	${Qt5Widgets_LIBRARIES}
	${Qt5X11Extras_LIBRARIES}
	${OGRE_LIBRARIES}
	#/usr/lib/x86_64-linux-gnu/OGRE-1.9.0/RenderSystem_GL.so
	${Festival_LIBRARIES}
	${EST_LIBRARIES}
	pthread
)

install(TARGETS FestivalSynthesizer RobotFace 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY images/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/images
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY mesh/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mesh
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ptu.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#	 target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
