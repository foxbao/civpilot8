cmake_minimum_required(VERSION 3.5)
project(civmap)
set(CMAKE_CXX_STANDARD 14)

if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Debug")
endif()

include(cmake/options.cmake)
# include(cmake/process_proto.cmake)
# include(${civauto_SOURCE_DIR}/cmake/process_all_proto.cmake)  #生成项目依赖的proto

include(cmake/functions.cmake)

add_subdirectory(core)

