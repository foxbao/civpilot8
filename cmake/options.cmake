# find_package(GTest REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(GLOG REQUIRED)
# find_package(OpenCV REQUIRED)
# find_package(Protobuf REQUIRED)


# set(INNER_PROTO_INTERFACE_DIR ${PROJECT_SOURCE_DIR}/proto)
# # set(INNER_PROTO_TARGET_NAME ${PROJECT_SOURCE_DIR}/_proto_target)
# set(INNER_PROTO_TARGET_NAME ${PROJECT_NAME}_proto_target)

string(TOUPPER ${CMAKE_SYSTEM_NAME}_${CMAKE_SYSTEM_PROCESSOR} output_dir_var)
set(THIRDPARTY_DIR ${PROJECT_SOURCE_DIR}/thirdparty/${output_dir_var})
# message(STATUS "!!!!!${THIRDPARTY_DIR}")