set(MODULE_NAME ${PROJECT_NAME}_component) 
add_library(${MODULE_NAME} SHARED)
set_cxx_standard(${MODULE_NAME})
target_compile_definitions(${MODULE_NAME} PRIVATE MODULE_NAME="civloc")


FILE( GLOB CPPS [^.]*.cpp [^.]*.cc)
FILE( GLOB HPPS [^.]*.hpp [^.]*.h)

target_sources(${MODULE_NAME}
PRIVATE
${HPPS}
${CPPS}
)

target_link_libraries(${MODULE_NAME}
PUBLIC
    ${INNER_PROTO_TARGET_NAME}
    ${PROJECT_NAME}_core
    cyber
    cyber_tool
)

target_include_directories(${MODULE_NAME} 
PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${COMMON_INTERFACE_DIR}
    ${PROJECT_SOURCE_DIR}
    ${COMMON_CORE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
)

add_subdirectory(tool)

