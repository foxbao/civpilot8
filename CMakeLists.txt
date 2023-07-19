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

find_package(glog REQUIRED)
find_package(Protobuf REQUIRED)
# pkg_check_modules(Protobuf REQUIRED protobuf)
# pkg_check_modules(Glog REQUIRED libglog)
pkg_check_modules(Gflags REQUIRED gflags)

find_package(absl REQUIRED)
find_package(PROJ4 REQUIRED)

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

include(cmake/macros.cmake)
#---- 导入选项参数 ----
include(cmake/options.cmake)
#---- 导入函数 ----
include(cmake/functions.cmake)

include(cmake/process_all_proto.cmake)  #生成项目依赖的proto


# set the default modules to be compiled
set(compile_dirs
    # message
    common
    cyber
    cyber_tool
    driver
    # map
    #thirdparty
)
# compile only the necessary modules
list(APPEND compile_dirs
  civmap
  civview
#   civloc
  # planning
  # routing
  # 
  # test
)

foreach(subdir ${compile_dirs})
    if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/${subdir}")
      add_subdirectory(${subdir})
    endif()
endforeach()