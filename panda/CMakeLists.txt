cmake_minimum_required(VERSION 3.10)
project (basebot)
SET(CMAKE_CXX_FLAGS "-std=c++17 -I/usr/include -I/usr/local/include -O3")
SET(CMAKE_BUILD_TYPE "Release")

# Mac OS X global build options
if(CMAKE_SYSTEM_NAME MATCHES Darwin)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/opt/homebrew/include")
endif()

# set common dependencies
# - eigen3
set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR})
if(NOT EIGEN3_INCLUDE_DIR)
	find_package(Eigen3 QUIET)
	# TODO: Find automatic way to find for Ubuntu 14.04
	if(NOT EIGEN3_INCLUDE_DIR)
		set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
	endif()
endif()
include_directories(${EIGEN3_INCLUDE_DIR})

# # - CHAI3D
find_package(CHAI3D REQUIRED)
include_directories(${CHAI3D_INCLUDE_DIRS})
add_definitions(${CHAI3D_DEFINITIONS})
if (CMAKE_SYSTEM_NAME MATCHES Darwin)
  add_definitions(-D_MACOSX -DMACOSX)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
  add_definitions(-D_LINUX -DLINUX)
endif () #NOTE: Windows is currently unsupported
link_directories(${CHAI3D_LIBRARY_DIRS})

# - SAI-SIMULATION
find_package(SAI-SIMULATION REQUIRED)
include_directories(${SAI-SIMULATION_INCLUDE_DIRS})
add_definitions(${SAI-SIMULATION_DEFINITIONS})

# - SAI-MODEL
FIND_PACKAGE(SAI-MODEL REQUIRED)
include_directories(${SAI-MODEL_INCLUDE_DIRS})
add_definitions(${SAI-MODEL_DEFINITIONS})

# - SAI-GRAPHICS
FIND_PACKAGE(SAI-GRAPHICS REQUIRED)
include_directories(${SAI-GRAPHICS_INCLUDE_DIRS})
add_definitions(${SAI-GRAPHICS_DEFINITIONS})

# - SAI-COMMON
find_package(SAI-COMMON REQUIRED)
include_directories(${SAI-COMMON_INCLUDE_DIRS})
add_definitions(${SAI-COMMON_DEFINITIONS})

# - SAI-URDF
find_package(SAI-URDF REQUIRED)
include_directories(${SAI-URDF_INCLUDE_DIRS})
add_definitions(${SAI-URDF_DEFINITIONS})

# - SAI-PRIMITIVES
find_package(SAI-PRIMITIVES REQUIRED)
include_directories(${SAI-PRIMITIVES_INCLUDE_DIRS})
add_definitions(${SAI-PRIMITIVES_DEFINITIONS})

# - SAI-INTERFACES
find_package(SAI-INTERFACES REQUIRED)
include_directories(${SAI-INTERFACES_INCLUDE_DIRS})

# - glfw3
find_package(glfw3 QUIET)
find_library(GLFW_LIBRARY glfw)

# - hiredis
find_library(HIREDIS_LIBRARY hiredis)


set(BASEBOT_COMMON_LIBRARIES
	${CHAI3D_LIBARIES}
	${SAI-INTERFACES_LIBRARIES}
	${HIREDIS_LIBRARY}
	${GLFW_LIBRARY}
	${SAI-GRAPHICS_LIBRARIES}
	${SAI-MODEL_LIBRARIES}
	${SAI-SIMULATION_LIBRARIES}
	${SAI-COMMON_LIBRARIES}
	${SAI-URDF_LIBRARIES}
	${CHAI3D_LIBRARIES}
	${HIREDIS_LIBRARY}
	${SAI-PRIMITIVES_LIBRARIES}
)

set(PANDA_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPANDA_FOLDER="${PANDA_FOLDER}")

# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ../bin/panda_basebot)
ADD_EXECUTABLE (simviz_panda_gripper simviz.cpp ${PROJECT_SOURCE_DIR})
#ADD_EXECUTABLE (controller_panda_gripper controller.cpp ${PROJECT_SOURCE_DIR})
ADD_EXECUTABLE (controller_catch controller_catcher.cpp ${PROJECT_SOURCE_DIR})
ADD_EXECUTABLE (simviz_Motiv simviz_Motiv.cpp ${PROJECT_SOURCE_DIR})

# and link the library against the executable
TARGET_LINK_LIBRARIES (simviz_panda_gripper ${BASEBOT_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_Motiv ${BASEBOT_COMMON_LIBRARIES})
#TARGET_LINK_LIBRARIES (controller_panda_gripper ${BASEBOT_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_catch ${BASEBOT_COMMON_LIBRARIES})
