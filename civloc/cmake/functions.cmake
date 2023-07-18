#硬编码了protoc可执行程序
set(THIRDPARTY_DIR_FOR_PROTOC "/project/thirdparty/X86_64" CACHE STRING "protoc and lib path")
set(ENV{LD_LIBRARY_PATH} ${THIRDPARTY_DIR_FOR_PROTOC}_protoc/lib)
# set(PROTOBUF_PROTOC_EXECUTABLE ${THIRDPARTY_DIR_FOR_PROTOC}_protoc/bin/protoc CACHE STRING "protoc cmd")
set(PROTOBUF_PROTOC_EXECUTABLE /usr/local/bin/protoc CACHE STRING "protoc cmd")

#用于获取某个目录下的所有子目录
macro(SUBDIRLIST result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

#用于检测cmake是否在docker环境中运行
function(detect_indocker)
  if(NOT EXISTS "/.indocker")
    message(FATAL_ERROR "Run cmake in docker!!!")
    return()
  endif()
endfunction()

function(generate_proto_files ret_proto_files) 
  # ---- 设置提取变量 ----
  set(prefix proto_interface)
  set(noValues)
  set(singleValues PROTO_OUTPUT_DIR PROTO_INPUT_DIR)
  set(multiValues PROTO_PATHS PROTO_FILES)

  # ---- 解析变量 ----
  include(CMakeParseArguments )
  cmake_parse_arguments(${prefix}
    "${noValues}"
    "${singleValues}"
    "${multiValues}"
    ${ARGN}
  )

  # ---- 一些变量前置检查 ----
  if(NOT proto_interface_PROTO_OUTPUT_DIR)
    message(FATAL_ERROR "Not define proto output dir !!!")
  else()
    message(STATUS "proto output dir is: ${proto_interface_PROTO_OUTPUT_DIR}")
  endif()

  if ((NOT proto_interface_PROTO_INPUT_DIR) AND (NOT EXISTS ${proto_interface_PROTO_INPUT_DIR}))
    message(FATAL_ERROR "Not define proto input dir !!!")
  else()
    message(STATUS "proto input dir is: ${proto_interface_PROTO_INPUT_DIR}")
  endif()

  if(NOT proto_interface_PROTO_PATHS)
    message(FATAL_ERROR "Not define proto paths !!!")
  else()
    message(STATUS "proto paths: ${proto_interface_PROTO_PATHS}")
  endif() 

  if(NOT proto_interface_PROTO_FILES)
    message(FATAL_ERROR "Not define proto files !!!")
  endif() 

  if (NOT EXISTS ${proto_interface_PROTO_OUTPUT_DIR})
    message(STATUS "Create proto output dir ${proto_interface_PROTO_OUTPUT_DIR}")
    file(MAKE_DIRECTORY ${proto_interface_PROTO_OUTPUT_DIR})
  endif()

  #construct cmd
  set(final_exec_cmd ${PROTOBUF_PROTOC_EXECUTABLE})
  foreach(proto_path_opt ${proto_interface_PROTO_PATHS})
    set(final_exec_cmd "${final_exec_cmd};--proto_path=${proto_path_opt}")
    set(final_python_exec_cmd "${final_exec_cmd}")
  endforeach()
  set(final_exec_cmd "${final_exec_cmd};--cpp_out=${proto_interface_PROTO_OUTPUT_DIR}")
  # set(final_python_exec_cmd "${final_python_exec_cmd};--python_out=${CMAKE_SOURCE_DIR}/output/python")
  set(final_python_exec_cmd "${final_python_exec_cmd};--python_out=${proto_interface_PROTO_OUTPUT_DIR}")
  foreach(proto_file_item ${proto_interface_PROTO_FILES})
    #检查输入文件是否存在
    if(NOT EXISTS ${proto_interface_PROTO_INPUT_DIR}/${proto_file_item})
      message(FATAL_ERROR "proto file not exist:  ${proto_interface_PROTO_INPUT_DIR}/${proto_file_item}")
    endif()

    #检查目标文件是否已经生成
    file(TO_NATIVE_PATH  ${proto_interface_PROTO_INPUT_DIR}/${proto_file_item} proto_native_path)
    string(REPLACE ".proto" ".pb.cc" proto_dest_file ${proto_native_path})
    #string(REPLACE "${proto_interface_PROTO_INPUT_DIR}" "${proto_interface_PROTO_OUTPUT_DIR}" proto_dest_file ${proto_dest_file})
    string(REGEX REPLACE "^${proto_interface_PROTO_INPUT_DIR}" "${proto_interface_PROTO_OUTPUT_DIR}" proto_dest_file ${proto_dest_file})
    if((NOT EXISTS ${proto_dest_file}) OR (${proto_native_path} IS_NEWER_THAN ${proto_dest_file}))
      execute_process(COMMAND ${final_exec_cmd} ${proto_native_path})
      #message(STATUS ${final_exec_cmd} == ${proto_native_path})
      execute_process(COMMAND ${final_python_exec_cmd} ${proto_native_path}) #liyuan 20210902 generate py file for python tools to use

    endif()
    list(APPEND proto_source_file_list ${proto_dest_file})
  endforeach()
  #返回pb.cc的文件列表
  set(${ret_proto_files} ${proto_source_file_list} PARENT_SCOPE)
endfunction()

#配置不同ping的c++属性
macro(set_cxx_standard target_name)
  if(NOT TARGET ${target_name})
    message(FATAL_ERROR "set_cxx_standard target name ${target_name} not defined")
  endif()
  target_compile_features(${target_name}
  PRIVATE
      $<$<STREQUAL:${CMAKE_SYSTEM_NAME},QNX>:cxx_std_14>
      $<$<STREQUAL:${CMAKE_SYSTEM_NAME},Linux>:cxx_std_17>
  )
endmacro()

#方便配置平台相关的属性
macro(set_compile_definition target_name)
  if(NOT TARGET ${target_name})
    message(FATAL_ERROR "set_compile_definition target name ${target_name} not defined")
  endif()
  target_compile_definitions(${target_name}
  PRIVATE
      $<$<STREQUAL:${CMAKE_SYSTEM_NAME},QNX>:__QNX__=1 _XOPEN_SOURCE=700 _QNX_SOURCE>
      $<$<STREQUAL:${CMAKE_SYSTEM_NAME},Linux>:__LINUX__=1>
  )
endmacro()


#方便安装与cyber相关的dag config launch目录
macro(install_config_dirs)
  foreach(dir_item ${ARGN})
    if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir_item})
      message(FATAL_ERROR "${CMAKE_CURRENT_SOURCE_DIR}/${dir_item} not exit !!!")
    endif()
  endforeach()

  file(RELATIVE_PATH _relative_dir ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
  install(
      DIRECTORY ${ARGN}
      DESTINATION ${_relative_dir}
  )
endmacro()


#方便安装目标的产出
macro(install_targets_files)
  install(
  TARGETS
      ${ARGN}
  ARCHIVE
      DESTINATION ${INSTALL_LIBDIR}
      COMPONENT lib
  RUNTIME
      DESTINATION ${INSTALL_BINDIR}
      COMPONENT bin
  LIBRARY
      DESTINATION ${INSTALL_LIBDIR}
      COMPONENT lib 
  )
endmacro()