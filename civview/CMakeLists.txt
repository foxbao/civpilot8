cmake_minimum_required(VERSION 3.5)

project(civview)

set(CMAKE_CXX_STANDARD 14)

if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Debug")
endif()

add_subdirectory(core)
# add_subdirectory(test)
add_subdirectory(middle_ware/cyber)