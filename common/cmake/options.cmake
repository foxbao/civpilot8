find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GLOG REQUIRED)
# find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)

set(INNER_PROTO_INTERFACE_DIR ${PROJECT_SOURCE_DIR}/proto)
# set(INNER_PROTO_TARGET_NAME ${PROJECT_SOURCE_DIR}/_proto_target)
set(INNER_PROTO_TARGET_NAME ${PROJECT_NAME}_proto_target)

