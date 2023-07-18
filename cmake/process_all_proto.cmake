set(PROTO_TARGET_NAME proto_msg)
#如果已经定义了proto的target就不在处理,防止重复生成proto target
if(TARGET ${PROTO_TARGET_NAME})
    return()
endif()
# message(STATUS "!!!!!!!!!!!${TARGET}")
# ${PROJECT_SOURCE_DIR}

file(GLOB_RECURSE all_proto_files ${civauto_SOURCE_DIR}/message/*.proto)
foreach(proto_item ${all_proto_files})
    string(REPLACE "${civauto_SOURCE_DIR}/" "" relative_proto_path ${proto_item})
    list(APPEND all_relative_proto_files ${relative_proto_path})
endforeach()

# message(STATUS "all_relative_proto_files=${all_relative_proto_files}")

generate_proto_files(
    proto_output_files_list
PROTO_OUTPUT_DIR
    ${civauto_SOURCE_DIR}
PROTO_INPUT_DIR
    ${civauto_SOURCE_DIR}
PROTO_PATHS
    ${civauto_SOURCE_DIR}
PROTO_FILES
    ${all_relative_proto_files}
)

set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}")
find_package(Protobuf REQUIRED)
if (NOT Protobuf_FOUND)
    message(FATAL_ERROR "Not found Protobuf library!!!")
endif()

add_library(${PROTO_TARGET_NAME} SHARED)
target_sources(${PROTO_TARGET_NAME} 
PRIVATE
    ${proto_output_files_list}
)

target_include_directories(${PROTO_TARGET_NAME} 
PRIVATE 
    ${Protobuf_INCLUDE_DIRS}
)
target_link_libraries(${PROTO_TARGET_NAME} 
  ${Protobuf_LIBRARIES}
)
target_compile_options(${PROTO_TARGET_NAME} 
PRIVATE 
  "-fPIC"
)

#安装配置
install(
TARGETS
    ${PROTO_TARGET_NAME} 
ARCHIVE
    DESTINATION ${INSTALL_LIBDIR}
    COMPONENT lib
RUNTIME
    DESTINATION ${INSTALL_BINDIR}
    COMPONENT bin
LIBRARY
    DESTINATION ${INSTALL_LIBDIR}
    COMPONENT lib 
)