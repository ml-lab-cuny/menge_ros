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
  geometry_msgs
  visualization_msgs
  tf
  message_generation
  genmsg
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
#find the correct OpenMP flag 
FIND_PACKAGE(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OpenMP_CXX_FLAGS} -fpermissive -DNDEBUG")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

FIND_PACKAGE(SDL REQUIRED)
if(NOT SDL_FOUND)
	message(STATUS "I will try to link against local libraries")
	INCLUDE_DIRECTORIES (${PROJECT_SOURCE_DIR}/include)
	LINK_DIRECTORIES  (${PROJECT_SOURCE_DIR}/lib)
	SET(SDL_LIBRARY "SDL")
	SET(SDLMAIN_LIBRARY "SDLmain")
else()
	INCLUDE_DIRECTORIES(${SDL_INCLUDE_DIR})
endif()

FIND_PACKAGE(SDL_ttf REQUIRED)
if(NOT SDLTTF_FOUND)
	MESSAGE(STATUS "I will try to link against local libraries")
	INCLUDE_DIRECTORIES (${PROJECT_SOURCE_DIR}/include)
	LINK_DIRECTORIES  (${PROJECT_SOURCE_DIR}/lib)
	SET(SDLTTF_LIBRARY "SDL_ttf")
else()
	INCLUDE_DIRECTORIES(${SDLTTF_INCLUDE_DIR})
endif()

FIND_PACKAGE(SDL_image REQUIRED)
if(NOT SDLIMAGE_FOUND)
	message(STATUS "I will try to link against local libraries")
	INCLUDE_DIRECTORIES (${PROJECT_SOURCE_DIR}/include)
	LINK_DIRECTORIES  (${PROJECT_SOURCE_DIR}/lib)
	SET(SDLIMAGE_LIBRARY SDL_image)
else()
	INCLUDE_DIRECTORIES(${SDLIMAGE_INCLUDE_DIR})
endif()

FIND_PACKAGE(OpenGL REQUIRED)
if(OPENGL_FOUND)
	INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR})
endif()

FIND_PACKAGE(PNG REQUIRED)

find_package(TinyXML REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES RunSim.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

file(GLOB_RECURSE Foo_HEADERS ${PROJECT_SOURCE_DIR}/include/*.h)
set (Foo_INCLUDE_DIRS "")
foreach (_headerFile ${Foo_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND Foo_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES Foo_INCLUDE_DIRS)


catkin_package(
  INCLUDE_DIRS ${Foo_INCLUDE_DIRS}
  LIBRARIES menge 
  CATKIN_DEPENDS cmake_modules roscpp rospy std_msgs geometry_msgs tf message_runtime
  DEPENDS OpenMP SDL SDL_ttf SDL_image OpenGL PNG TinyXML
)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Foo_INCLUDE_DIRS}
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
	menge
	${source_files}
)

TARGET_LINK_LIBRARIES(menge ${TinyXML_LIBRARIES} ${OPENGL_LIBRARIES} ${SDL_LIBRARY} ${SDLIMAGE_LIBRARY} ${SDLTTF_LIBRARY} ${PNG_LIBRARY} ${catkin_LIBRARIES})


