#include(${PROJECT_SOURCE_DIR}/../cmake/functions.cmake)


# # protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/fusion.proto)
# file(GLOB_RECURSE inner_proto_files "${INNER_PROTO_INTERFACE_DIR}/*.proto")
# # file(GLOB_RECURSE inner_proto_files "${INNER_PROTO_INTERFACE_DIR}/*.pro")

# foreach(inner_proto_abs ${inner_proto_files})
#     get_filename_component(inner_proto_rel ${inner_proto_abs} NAME)
#     list(APPEND inner_proto_rel_files ${PROJECT_NAME}/proto/${inner_proto_rel})
#     # message(STATUS "!!!!!!!!!!!!!!!! proto/${inner_proto_rel}")
# endforeach()

# ---- 自定义proto文件的生成 ----
file(GLOB_RECURSE inner_proto_files "${INNER_PROTO_INTERFACE_DIR}/*.proto")
foreach(inner_proto_abs ${inner_proto_files})
    get_filename_component(inner_proto_rel ${inner_proto_abs} NAME)
    list(APPEND inner_proto_rel_files ${PROJECT_NAME}/proto/${inner_proto_rel})
    # message(status !!!!!!!!!!!!!!!!${PROJECT_NAME}/proto/${inner_proto_rel})
endforeach()



generate_proto_files(
    custom_proto_file_list
PROTO_OUTPUT_DIR
    ${civauto_BINARY_DIR}
PROTO_INPUT_DIR
    ${civauto_SOURCE_DIR}
PROTO_PATHS
    ${civauto_SOURCE_DIR}
PROTO_FILES
    ${inner_proto_rel_files}
)


# ---- 生成 object目标 ----

set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${THIRDPARTY_DIR}")
find_package(Protobuf REQUIRED)
if (NOT Protobuf_FOUND)
    message(FATAL_ERROR "Not found Protobuf library!!!")
endif()

add_library(${INNER_PROTO_TARGET_NAME} OBJECT)
# message(STATUS "PROTO_TARGET_NAME=${PROTO_TARGET_NAME}")
target_sources(${INNER_PROTO_TARGET_NAME} 
PRIVATE 
    ${custom_proto_file_list} #自定义proto
    ${proto_output_files_list} #系统接口proto
)


target_include_directories(${INNER_PROTO_TARGET_NAME} 
PRIVATE 
    ${PROJECT_BINARY_DIR}
    ${Protobuf_INCLUDE_DIRS}
)
target_link_libraries(${INNER_PROTO_TARGET_NAME} PUBLIC ${Protobuf_LIBRARIES})
target_compile_options(${INNER_PROTO_TARGET_NAME} PRIVATE "-fPIC")

