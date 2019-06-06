#
# This file is part of IRA.
#
# Created by Alvaro Parra on 19/3/19.
# Copyright © 2019 Alvaro Parra <alvaro dot parrabustos at adelaide
# dot edu dot au> (The University of Adelaide)
# For more information see <https://github.com/ajparra/IRA>
#
# This work was supported by Maptek (http://maptek.com) and the
# ARC Grant DP160103490.
#
# IRA is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# IRA is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with IRA. If not, see <http://www.gnu.org/licenses/>.
#


cmake_minimum_required (VERSION 3.0)
project (RAL)

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
list(APPEND CMAKE_MODULE_PATH "${RAL_SOURCE_DIR}/cmake")
include(UpdateCacheVariable)



find_package (Eigen3 3.3 REQUIRED NO_MODULE)


find_package(SuiteSparse)

if (SUITESPARSE_FOUND)
    # On Ubuntu the system install of SuiteSparse (v3.4.0) up to at least
    # Ubuntu 13.10 cannot be used to link shared libraries.
    if (BUILD_SHARED_LIBS AND
        SUITESPARSE_IS_BROKEN_SHARED_LINKING_UBUNTU_SYSTEM_VERSION)

        message(FATAL_ERROR "You are attempting to build Ceres as a shared "
        "library on Ubuntu using a system package install of SuiteSparse "
        "3.4.0. This package is broken and does not support the "
        "construction of shared libraries (you can still build Ceres as "
        " a static library).  If you wish to build a shared version of Ceres "
        "you should uninstall the system install of SuiteSparse "
        "(libsuitesparse-dev) and perform a source install of SuiteSparse "
        "(we recommend that you use the latest version), "
        "see http://ceres-solver.org/building.html for more information.")
    endif (BUILD_SHARED_LIBS AND
           SUITESPARSE_IS_BROKEN_SHARED_LINKING_UBUNTU_SYSTEM_VERSION)

    message("-- Found SuiteSparse ${SUITESPARSE_VERSION}, "
        "building with SuiteSparse.")
else (SUITESPARSE_FOUND)
    message(FATAL_ERROR "Unable to find SuiteSparse!")
endif (SUITESPARSE_FOUND)


link_directories( ${SUITESPARSE_LIBRARY_DIR_HINTS} )


add_executable( l1_irls main.cpp l1_irls.cpp )

#message(" SuiteSparse INCLUDE ${SUITESPARSE_INCLUDE_DIRS}, "
#"building with SuiteSparse.")



target_include_directories( l1_irls PUBLIC
 ${Eigen3_INCLUDE_DIRS}
 ${SUITESPARSE_INCLUDE_DIRS}
)


target_link_libraries (l1_irls Eigen3::Eigen ${SUITESPARSE_LIBRARIES} umfpack)
