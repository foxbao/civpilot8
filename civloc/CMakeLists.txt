cmake_minimum_required(VERSION 3.5)
project(civloc)
set(CMAKE_CXX_STANDARD 14)

if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Debug")
endif()

# platform control
if(TARGET_VENDER STREQUAL MDC)
    message("---mdc not build zloc/view----")
elseif(CMAKE_SYSTEM_NAME STREQUAL Linux)
    set(BUILD_VIEW TRUE)
    add_definitions(-D__BUILD_VIEW_)
    set(BUILD_FILESYSTEM TRUE)
    add_definitions(-D__BUILD_FILESYSTEM_)
    set(BUILD_TOOL TRUE)
endif()

include(cmake/options.cmake)
include(cmake/process_proto.cmake)
# include(${civauto_SOURCE_DIR}/cmake/process_all_proto.cmake)  #生成项目依赖的proto

include(cmake/functions.cmake)

include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})

# ---- 添加源码 ----
include_directories(core/src)
add_subdirectory(core)

add_subdirectory(middle_ware/cyber)

set(target_name test)
add_executable(${target_name} civloc_test.cpp)
target_link_libraries(
	${target_name}
	PRIVATE
	common
	${PROJECT_NAME}_core 
	${PROJECT_NAME}_component 
	glog::glog
	${GFLAGS_LIBRARIES}
	)