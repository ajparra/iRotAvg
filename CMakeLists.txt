#
# This file is part of iRotAvg.
#
# Created by Alvaro Parra on 19/3/19.
# Copyright © 2019 Alvaro Parra <alvaro dot parrabustos at adelaide
# dot edu dot au> (The University of Adelaide)
# For more information see <https://github.com/ajparra/iRotAvg>
#
# This work was supported by Maptek (http://maptek.com) and the
# ARC Grant DP160103490.
#
# iRotAvg is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# iRotAvg is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with iRotAvg. If not, see <http://www.gnu.org/licenses/>.
#


cmake_minimum_required (VERSION 3.0)
project (irotavg)

set(default_build_type "Release")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
        STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()


set (CMAKE_CXX_STANDARD 11)

# Make CMake aware of the cmake folder for local FindXXX scripts,
# append rather than set in case the user has passed their own
# additional paths via -D.
list(APPEND CMAKE_MODULE_PATH "${irotavg_SOURCE_DIR}/ral/cmake")
include(UpdateCacheVariable)

message( "irotavg_SOURCE_DIR:    ${irotavg_SOURCE_DIR}" )

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_subdirectory(src)
