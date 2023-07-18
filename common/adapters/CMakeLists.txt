FILE( GLOB CPPS [^.]*.cpp [^.]*.cc)
FILE( GLOB HPPS [^.]*.hpp [^.]*.h [^.]*.inl)
FILE( GLOB EXCEPT_FILES *test.cc)

LIST(LENGTH EXCEPT_FILES NOTEMPTY)
if (NOTEMPTY)
    list(REMOVE_ITEM CPPS ${EXCEPT_FILES})
endif()

target_sources(${MODULE_NAME}
PUBLIC
${HPPS}
${CPPS}
)