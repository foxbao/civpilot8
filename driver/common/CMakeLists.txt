set(MODULE_NAME ${PROJECT_NAME}_common)
add_library(${MODULE_NAME} SHARED)


FILE( GLOB CPPS [^.]*.cpp [^.]*.cc)
FILE( GLOB HPPS [^.]*.hpp [^.]*.h)

target_sources(${MODULE_NAME}
PRIVATE
    ${HPPS}
    ${CPPS}
)

target_include_directories(${MODULE_NAME}
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(${MODULE_NAME}
    PRIVATE
)

add_subdirectory(parser)
add_subdirectory(stream)