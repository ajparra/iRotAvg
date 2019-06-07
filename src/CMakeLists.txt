c #
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

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/ral/cmake")
include(UpdateCacheVariable)


find_package(Boost COMPONENTS filesystem REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

find_package(OpenCV 4 REQUIRED)
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


add_executable( ira IRA
    ${PROJECT_SOURCE_DIR}/ral/l1_irls
    ${PROJECT_SOURCE_DIR}/third_party/DBoW2/DBoW2/FeatureVector
    ${PROJECT_SOURCE_DIR}/third_party/DBoW2/DBoW2/BowVector
    ${PROJECT_SOURCE_DIR}/third_party/DBoW2/DBoW2/ScoringObject
    ${PROJECT_SOURCE_DIR}/third_party/DBoW2/DBoW2/FORB
    ${PROJECT_SOURCE_DIR}/third_party/DBoW2/DUtils/Timestamp
    ${PROJECT_SOURCE_DIR}/third_party/DBoW2/DUtils/Random
    ORBVocabulary
    ViewGraph
    SequenceLoader
    Frame
    View
    Camera
    ORBExtractor
    Pose
    Converter
    ViewDatabase
)

target_include_directories( ira PUBLIC
    ${Eigen3_INCLUDE_DIRS}
    ${SUITESPARSE_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/third_party/DBoW2/DBoW2/
    ${PROJECT_SOURCE_DIR}/ral/
)

target_link_libraries (ira
    Eigen3::Eigen
    ${SUITESPARSE_LIBRARIES} umfpack
    ${OpenCV_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY} 
    -lboost_system)

