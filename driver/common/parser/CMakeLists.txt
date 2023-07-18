FILE( GLOB CPPS [^.]*.cpp [^.]*.cc [^.]*.c)
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