# 
#  Copyright (C) 2014 Walkman
#  Author: 
#  email: 
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>
#

cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)
include(ExternalProject)
project(centralized_inverse_kinematics)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(idynutils REQUIRED)
FIND_PACKAGE(GYM REQUIRED)
FIND_PACKAGE(paramHelp REQUIRED)
FIND_PACKAGE(kdl_codyco REQUIRED)
FIND_PACKAGE(iDynTree REQUIRED)
FIND_PACKAGE(PCL 1.7 REQUIRED)
FIND_PACKAGE(OpenSoT REQUIRED)
FIND_PACKAGE(roscpp REQUIRED)
FIND_PACKAGE(moveit_core REQUIRED)
FIND_PACKAGE(sensor_msgs REQUIRED)

# add include directories
INCLUDE_DIRECTORIES(include 
		    ${YARP_INCLUDE_DIRS} 
		    ${idynutils_INCLUDE_DIRS}
		    ${GYM_INCLUDE_DIRS}
		    ${kdl_codyco_INCLUDE_DIRS} 
                    ${PCL_INCLUDE_DIRS}
                    ${OpenSoT_INCLUDE_DIRS}
                    ${roscpp_INCLUDE_DIRS}
                    ${sensor_msgs_INCLUDE_DIRS}
                    ${moveit_core_INCLUDE_DIRS}
		    )

ADD_DEFINITIONS(${PCL_DEFINITIONS})
		    
ADD_EXECUTABLE(${PROJECT_NAME} src/centralized_inverse_kinematics_main.cpp 
			          src/centralized_inverse_kinematics_thread.cpp
                               src/problems/simple_problem.cpp
src/problems/interaction_problem.cpp
			          )

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${iDynTree_LIBRARIES}
                                      ${YARP_LIBRARIES}
				      ${idynutils_LIBRARIES}
				      ${GYM_LIBRARIES}
                                      ${OpenSoT_LIBRARIES}
                                      ${roscpp_LIBRARIES}
                                      ${sensor_msgs_LIBRARIES}
                                      ${moveit_core_LIBRARIES}
				      )

add_subdirectory(app)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
