# set( MODULE_NAME ${PROJECT_NAME})
FILE( GLOB CPPS [^.]*.cpp [^.]*.cc)
FILE( GLOB HPPS [^.]*.hpp [^.]*.h)

target_sources(${MODULE_NAME}
PUBLIC
${HPPS}
${CPPS}
)
