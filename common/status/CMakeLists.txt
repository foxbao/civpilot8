# set( MODULE_NAME ${PROJECT_NAME}_status)
FILE( GLOB CPPS [^.]*.cpp [^.]*.cc)
FILE( GLOB HPPS [^.]*.hpp [^.]*.h)

# add_library(${MODULE_NAME} SHARED)
target_sources(${MODULE_NAME}
PRIVATE
${HPPS}
${CPPS}
)



