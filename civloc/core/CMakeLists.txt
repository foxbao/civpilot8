set(MODULE_NAME ${PROJECT_NAME}_core)

add_library(${MODULE_NAME} SHARED)
add_subdirectory(src)
add_subdirectory(imu_gps_localizer)

FILE( GLOB CPPS [^.]*.cpp [^.]*.cc)
FILE( GLOB HPPS [^.]*.hpp [^.]*.h)

target_sources(${MODULE_NAME}
PRIVATE
    ${HPPS}
    ${CPPS}
)
# set the include path
target_include_directories(${MODULE_NAME}
PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}/../..
)

TARGET_LINK_LIBRARIES(${MODULE_NAME} 
PUBLIC
    ${INNER_PROTO_TARGET_NAME}
    proto_msg
    ${Protobuf_LIBRARIES}
PRIVATE
    common
    glog::glog 
    Geographiccc
)

if(BUILD_FILESYSTEM)
    target_link_libraries(${MODULE_NAME} PRIVATE stdc++fs)
endif()

