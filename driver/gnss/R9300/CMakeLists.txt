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

add_subdirectory(messages)
add_subdirectory(parser)

