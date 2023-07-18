set(MODULE_NAME ${PROJECT_NAME}_gnss)
add_library(${MODULE_NAME} SHARED)

file(GLOB CPPS [^.]*.cpp [^.]*.cc)
file(GLOB HPPS [^.]*.hpp [^.]*.h)
file(GLOB EXCEPT_FILES *test.cpp)

list(LENGTH EXCEPT_FILES NOTEMPTY)
if(NOTEMPTY)
  list(REMOVE_ITEM CPPS ${EXCEPT_FILES})
endif()

target_sources(${MODULE_NAME}
PRIVATE 
${HPPS} 
${CPPS}
)

target_link_libraries(
    ${MODULE_NAME} 
    PRIVATE 
    ${PROJECT_NAME}_common 
    proto_msg
    PUBLIC 
    cyber
    )

add_subdirectory(R9300)
add_subdirectory(Ublox8)

set(target_name gnss_component_test) # target name
add_executable(
    ${target_name} 
    gnss_component_test.cpp
    )

target_link_libraries(
  ${target_name} 
  PRIVATE 
  ${PROJECT_NAME}_gnss
  )


