cmake_minimum_required(VERSION 3.5)
project(civauto)
set(CMAKE_CXX_STANDARD 14)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(fastrtps REQUIRED)
find_package(fastcdr REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)
pkg_check_modules(Protobuf REQUIRED protobuf)
pkg_check_modules(Glog REQUIRED libglog)
pkg_check_modules(Gflags REQUIRED gflags)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${fastcdr_INCLUDE_DIR}
    ${fastrtps_INCLUDE_DIR}
    ${Protobuf_INCLUDE_DIRS}
    ${Glog_INCLUDE_DIRS}
    ${Gflags_INCLUDE_DIRS}
)

link_directories(
    ${fastcdr_LIB_DIR}
    ${fastrtps_LIB_DIR}
    ${Protobuf_LIBRARY_DIRS}
    ${Glog_LIBRARY_DIRS}
    ${Gflags_LIBRARY_DIRS}
)


# set the default modules to be compiled
set(compile_dirs
    # message
    #common
    cyber
    #cyber_tool
    #driver
    # map
    #thirdparty
)

foreach(subdir ${compile_dirs})
    if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/${subdir}")
      add_subdirectory(${subdir})
    endif()
endforeach()