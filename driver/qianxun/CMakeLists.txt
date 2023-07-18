set(MODULE_NAME ${PROJECT_NAME}_qianxun) 
add_library(${MODULE_NAME} SHARED)
FILE( GLOB CPPS [^.]*.cpp [^.]*.cc)
FILE( GLOB HPPS [^.]*.hpp [^.]*.h [^.]*.inl)
FILE( GLOB EXCEPT_FILES *test.cc *test.cpp)

LIST(LENGTH EXCEPT_FILES NOTEMPTY)
if (NOTEMPTY)
    list(REMOVE_ITEM CPPS ${EXCEPT_FILES})
endif()

target_sources(${MODULE_NAME}
PRIVATE
    ${HPPS}
    ${CPPS}
)

target_include_directories(${MODULE_NAME}
PUBLIC
${CMAKE_CURRENT_LIST_DIR}
${THIRDPARTY_DIR}
)

add_subdirectory(nosr)

# add_library( qx_lib SHARED IMPORTED )
# set_target_properties( 
#     qx_lib 
#     PROPERTIES IMPORTED_LOCATION 
#     ${THIRDPARTY_DIR}/qianxun/lib/debug/libqxwz-pssdk-1.9.1-20210425-qa-x86-64-linux-gcc-4.8.4-default-production-debug.so
#     )

target_link_libraries(${MODULE_NAME}
    PUBLIC
    PRIVATE
    common
    ${PROJECT_NAME}_common
    ${COMMON_DIR}/qianxun/lib/debug/libqxwz-pssdk-1.9.1-20210425-qa-x86-64-linux-gcc-4.8.4-default-production-debug.so
    # qx_lib
    # civ_thirdparty_X86_64
    proto_msg
)

# test program
set(target_name qianxun_app) 
add_executable(${target_name} qianxun_test.cpp)

target_include_directories(
    ${target_name} 
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}
    )

target_link_libraries(
    ${target_name}
    PRIVATE
    ${PROJECT_NAME}_gnss
    ${PROJECT_NAME}_qianxun
    # ${PROJECT_NAME}_common
    )



