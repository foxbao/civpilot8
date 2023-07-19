set(MODULE_NAME ${PROJECT_NAME}_core) 

FILE( GLOB CPPS [^.]*.cpp [^.]*.cc)
FILE( GLOB HPPS [^.]*.hpp [^.]*.h)

add_library(${MODULE_NAME} SHARED)
target_sources(${MODULE_NAME}
PRIVATE
    ${HPPS}
    ${CPPS}
)

target_include_directories(${MODULE_NAME} 
PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_LIST_DIR}
)
add_subdirectory(src)