cmake_minimum_required(VERSION 3.5)
project(common)
set(CMAKE_CXX_STANDARD 14)

if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Debug")
endif()

find_package(PROJ4 REQUIRED)
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
# include(cmake/process_proto.cmake)
include(../cmake/functions.cmake)


include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})


FILE( GLOB CPPS [^.]*.cpp [^.]*.cc)
FILE( GLOB HPPS [^.]*.hpp [^.]*.h)

set( MODULE_NAME ${PROJECT_NAME})


add_library(${MODULE_NAME} SHARED)

target_include_directories(${MODULE_NAME}
PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${PROJECT_SOURCE_DIR}
)

target_sources(${PROJECT_NAME}
PRIVATE
${HPPS}
${CPPS}
)

SUBDIRLIST(dir_list ${CMAKE_CURRENT_SOURCE_DIR})
foreach(subdir ${dir_list})
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${subdir}/CMakeLists.txt)
        add_subdirectory(${subdir})
    endif()
endforeach()

target_link_libraries(
	${PROJECT_NAME} 
	PUBLIC
	cyber
	Geographiccc
	proj
	absl::algorithm
	absl::base
	absl::debugging
	absl::flat_hash_map
	absl::flags
	absl::memory
	absl::meta
	absl::numeric
	absl::random_random
	absl::strings
	absl::synchronization
	absl::time
	absl::utility
PRIVATE
	proto_msg
	glog::glog
)

if(BUILD_FILESYSTEM)
    target_link_libraries(${PROJECT_NAME} PRIVATE stdc++fs)
endif()
