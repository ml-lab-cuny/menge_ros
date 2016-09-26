cmake_minimum_required(VERSION 2.8.3)
project(menge_core)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  std_msgs
)

find_package(TinyXML REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/
  LIBRARIES menge_core
  CATKIN_DEPENDS cmake_modules roscpp rospy std_msgs
)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/menge_core/
  ${PROJECT_SOURCE_DIR}/src/
  ${PROJECT_SOURCE_DIR}/src/BFSM/
  ${PROJECT_SOURCE_DIR}/src/viewer/
  ${PROJECT_SOURCE_DIR}/src/SceneGraph/
  ${PROJECT_SOURCE_DIR}/src/Math/
  ${PROJECT_SOURCE_DIR}/src/Agents/
  ${PROJECT_SOURCE_DIR}/src/Runtime/
  ${PROJECT_SOURCE_DIR}/src/PluginEngine/
  ${PROJECT_SOURCE_DIR}/src/resources/
  ${roscpp_INCLUDE_DIRS}
)

set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fpermissive" )

## Declare a C++ library
file(
	GLOB_RECURSE
	source_files
	${PROJECT_SOURCE_DIR}/*.cpp
	${PROJECT_SOURCE_DIR}/*.h
)


add_library(
	menge_core
	${source_files}
)

TARGET_LINK_LIBRARIES(menge_core 
    ${TinyXML_LIBRARIES}  
)

