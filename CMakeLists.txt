cmake_minimum_required(VERSION 2.8.3)
project(imu_allan_variance)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")


# ADD_THIRD_PARTY defines a named third party library, with included
# headers and linked libraries.
function(ADD_THIRD_PARTY lib_name includes # link_libs
        )
    # Skip if library exists, generally because of building sources
    if (TARGET ${lib_name})
        return()
    endif ()
    # Add interface
    add_library(${lib_name} INTERFACE)
    # Target includes
    target_include_directories(${lib_name} INTERFACE ${includes})
    # Link libraries
    target_link_libraries(${lib_name} INTERFACE ${ARGN})
endfunction()


find_package(Eigen3 REQUIRED)

find_package(Ceres REQUIRED)

#find_package(Boost REQUIRED COMPONENTS filesystem)
#if (Boost_FOUND)
#    include_directories(${Boost_INCLUDE_DIRS})
#    MESSAGE(STATUS "Boost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}.")
#    MESSAGE(STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}.")
#    MESSAGE(STATUS "Boost_LIB_VERSION = ${Boost_LIB_VERSION}.")
#    MESSAGE(STATUS "Boost_LIB_DIR = ${Boost_LIB_DIR}.")
#else ()
#    MESSAGE(FATAL_ERROR "Boost not found")
#endif ()
#
#set(Boost_INCLUDE_DIRS "${Boost_INCLUDE_DIRS}")
#set(THIRDPARTY_LIBRARY "/usr/local/lib")

## Boost (filesystem, system and serialization libraries only)
#add_third_party(boost ${Boost_INCLUDE_DIRS}
#        ${THIRDPARTY_LIBRARY}/libboost_filesystem.so.1.75.0
#        ${THIRDPARTY_LIBRARY}/libboost_system.so.1.75.0
#        ${THIRDPARTY_LIBRARY}/libboost_serialization.so.1.75.0) # todo(congyu) only works on my computer

include_directories(
        ${CERES_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        include
)

set(ACC_LIB_SOURCE_FILES
        src/acc_lib/allan_acc.cpp
        src/acc_lib/fitallan_acc.cpp
        )
set(GYR_LIB_SOURCE_FILES
        src/gyr_lib/allan_gyr.cpp
        src/gyr_lib/fitallan_gyr.cpp
        )

add_executable(imu_allan_variance
        src/imu_allan_variance.cpp
        ${GYR_LIB_SOURCE_FILES}
        ${ACC_LIB_SOURCE_FILES}
        )

target_link_libraries(imu_allan_variance
        ${CERES_LIBRARIES}
        # boost
        )
