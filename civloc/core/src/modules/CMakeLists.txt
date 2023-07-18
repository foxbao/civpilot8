FILE( GLOB CPPS [^.]*.cpp [^.]*.cc)
FILE( GLOB HPPS [^.]*.hpp [^.]*.h)

target_sources(${MODULE_NAME}
PRIVATE
${HPPS}
${CPPS}
)

add_subdirectory(common)
add_subdirectory(detect)
add_subdirectory(init)
add_subdirectory(loc)
add_subdirectory(message_cache)
add_subdirectory(predict)
add_subdirectory(state_server)

