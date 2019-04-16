cmake_minimum_required (VERSION 3.0)
project (linfslam)

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
list(APPEND CMAKE_MODULE_PATH "${linfslam_SOURCE_DIR}/l1_irls/cmake")
include(UpdateCacheVariable)


message( "linfslam_SOURCE_DIR:    ${linfslam_SOURCE_DIR}" )


find_package (Eigen3 3.3 REQUIRED NO_MODULE)
message( "AAAAAA Eigen inc dirs:    ${Eigen3_INCLUDE_DIRS} ppp" )





#add_subdirectory(l1_irls)

add_subdirectory(src)

