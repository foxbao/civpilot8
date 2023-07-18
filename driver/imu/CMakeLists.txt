set(MODULE_NAME ${PROJECT_NAME}_imu)
add_library(${MODULE_NAME} SHARED)

file(GLOB CPPS [^.]*.cpp [^.]*.cc)
file(GLOB HPPS [^.]*.hpp [^.]*.h [^.]*.inl)
file(GLOB EXCEPT_FILES *test.cpp)

list(LENGTH EXCEPT_FILES NOTEMPTY)
if(NOTEMPTY)
  list(REMOVE_ITEM CPPS ${EXCEPT_FILES})
endif()

target_sources(
  ${MODULE_NAME} 
  PRIVATE 
  ${HPPS} 
  ${CPPS})

target_link_libraries(
  ${MODULE_NAME}
  PRIVATE 
  common
  ${PROJECT_NAME}_common 
  
  proto_msg
  PUBLIC 
  cyber
  )

add_subdirectory(wheeltecN100)

# the serial testing program
set(target_name wheeltec_component) # target name
add_executable(
  ${target_name} 
  wheeltec_component_test.cpp
  )

target_link_libraries(
  ${target_name} 
  PRIVATE 
  ${PROJECT_NAME}_imu
  )
  