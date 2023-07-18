FILE( GLOB CPPS [^.]*.cpp [^.]*.cc)
FILE( GLOB HPPS [^.]*.hpp [^.]*.h)

FILE( GLOB EXCEPT_FILES offline.cpp)

LIST(LENGTH EXCEPT_FILES NOTEMPTY)
if (NOTEMPTY)
    list(REMOVE_ITEM CPPS ${EXCEPT_FILES})
endif()

target_sources(${MODULE_NAME}
PRIVATE
${HPPS}
${CPPS}
)





set(target_name ${PROJECT_NAME}_component_offline) 

add_executable(${target_name}
  ${CMAKE_CURRENT_LIST_DIR}/offline.cpp
)

target_include_directories(${target_name}
PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${COMMON_INTERFACE_DIR}
  ${THIRDPARTY_DIR}/cyber/include
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR}
)

target_link_directories(${target_name} 
  PRIVATE
    ${THIRDPARTY_DIR}/cyber/lib
    ${THIRDPARTY_DIR}/lib
)

target_link_libraries(${target_name}
PUBLIC
    # ${INNER_PROTO_TARGET_NAME}
    # proto_msg
    # ${PROJECT_NAME}_core
    ${PROJECT_NAME}_component
    ${PROJECT_NAME}_core
    cyber
    # glog
    common
)

