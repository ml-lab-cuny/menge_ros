cmake_minimum_required(VERSION 2.8.3)
project(menge_sim)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  menge_core
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
#find the correct OpenMP flag
FIND_PACKAGE(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OpenMP_CXX_FLAGS} -fpermissive -DNDEBUG")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

FIND_PACKAGE(SDL)
if(NOT SDL_FOUND)
	message(STATUS "I will try to link against local libraries")
	INCLUDE_DIRECTORIES (${PROJECT_SOURCE_DIR}/include)
	LINK_DIRECTORIES  (${PROJECT_SOURCE_DIR}/lib)
	SET(SDL_LIBRARY "SDL")
	SET(SDLMAIN_LIBRARY "SDLmain")
else()
	INCLUDE_DIRECTORIES(${SDL_INCLUDE_DIR})
endif()

FIND_PACKAGE(SDL_ttf)
if(NOT SDLTTF_FOUND)
	MESSAGE(STATUS "I will try to link against local libraries")
	INCLUDE_DIRECTORIES (${PROJECT_SOURCE_DIR}/include)
	LINK_DIRECTORIES  (${PROJECT_SOURCE_DIR}/lib)
	SET(SDLTTF_LIBRARY "SDL_ttf")
else()
	INCLUDE_DIRECTORIES(${SDLTTF_INCLUDE_DIR})
endif()

FIND_PACKAGE(SDL_image)
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


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
  INCLUDE_DIRS include
  CATKIN_DEPENDS menge_core roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${menge_core_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(menge_sim
#   src/${PROJECT_NAME}/menge_sim.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(menge_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
file(
	GLOB_RECURSE
	source_files
	${PROJECT_SOURCE_DIR}/src/*.cpp  
	${PROJECT_SOURCE_DIR}/inc/menge_sim/*.h
)

add_executable(
	menge_sim
	${source_files}
)

target_link_libraries (menge_sim menge ${OPENGL_LIBRARIES} ${SDL_LIBRARY} ${SDLIMAGE_LIBRARY} ${SDLTTF_LIBRARY} ${PNG_LIBRARY}

${catkin_LIBRARIES})

#file( 
#  GLOB
#  EXTRA_FILES
#  ${PROJECT_SOURCE_DIR}/linux/*
#  ${PROJECT_SOURCE_DIR}/common/*
#)

#file( COPY ${EXTRA_FILES} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} )




## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(menge_sim_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(menge_sim_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS menge_sim menge_sim_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY 
   common/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY 
   linux/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES 
   common/arial.ttf
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_menge_sim.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
