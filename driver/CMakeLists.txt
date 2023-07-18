cmake_minimum_required(VERSION 3.5)
project(civ_drivers)
set(CMAKE_CXX_STANDARD 14)

if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Debug")
endif()

include(cmake/options.cmake)
# include(cmake/process_proto.cmake)
include(../cmake/functions.cmake)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(dir_list
	common
	gnss
	imu
	qianxun
	# interface_qt
)

foreach(item ${dir_list})
    add_subdirectory(${item})
endforeach(item ${dir_list})
