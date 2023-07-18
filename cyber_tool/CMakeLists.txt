cmake_minimum_required(VERSION 3.5)
project(cyber_tool)
set(CMAKE_CXX_STANDARD 14)

if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Debug")
endif()

set(MODULE_NAME ${PROJECT_NAME})
add_library(${MODULE_NAME} SHARED)
FILE( GLOB CPPS [^.]*.cpp [^.]*.cc)
FILE( GLOB HPPS [^.]*.hpp [^.]*.h)

target_sources(${MODULE_NAME}
PRIVATE
${HPPS}
${CPPS}
)
target_link_libraries(${MODULE_NAME}
PRIVATE
    common
    cyber
)




