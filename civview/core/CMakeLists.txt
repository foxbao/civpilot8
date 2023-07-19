set(MODULE_NAME ${PROJECT_NAME}_core) 

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check http://doc.qt.io/qt-5/deployment-android.html for more information.
# They need to be set before the find_package(Qt5 ...) call.

find_package(Qt5 COMPONENTS Widgets REQUIRED)

FILE( GLOB CPPS [^.]*.cpp [^.]*.cc)
FILE( GLOB HPPS [^.]*.hpp [^.]*.h [^.]*.inl)
FILE( GLOB EXCEPT_FILES main.cpp *test.cpp)

LIST(LENGTH EXCEPT_FILES NOTEMPTY)
if (NOTEMPTY)
    list(REMOVE_ITEM CPPS ${EXCEPT_FILES})
endif()


find_package(Eigen3 REQUIRED)
if((Eigen3_FOUND) AND (DEFINED EIGEN3_INCLUDE_DIR))
 message(STATUS "Found Eigen3: " ${EIGEN3_INCLUDE_DIR})
 INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
else()
 message(FATAL_ERROR "EIGEN3 not found.")
endif()

find_package(VTK REQUIRED)
if(VTK_FOUND)
  message(STATUS "found VTK_DIR:${VTK_DIR}")
else(VTK_FOUND)
  message(FATAL_ERROR "VTK not found. Please set VTK_DIR")
endif(VTK_FOUND)

find_package(Qt5 COMPONENTS Core Gui Widgets OpenGL REQUIRED)
if(Qt5_FOUND)
  message(STATUS "found Qt5_DIR:${Qt5_DIR}")
else(Qt5_FOUND)
  message(FATAL_ERROR "Qt5 not found. Please set Qt5_DIR")
endif(Qt5_FOUND)

add_executable(${MODULE_NAME}
  main.cpp
  ${CPPS}
  ${HPPS}
  mainwindow.ui
)

target_include_directories(
    ${MODULE_NAME}
    PUBLIC
    ${VTK_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${SOURCE_DIR}
)
target_link_libraries(${MODULE_NAME} 
  PRIVATE 
  Qt5::Core 
  Qt5::Gui
  Qt5::Widgets 
  Qt5::OpenGL 
  
  ${VTK_LIBRARIES}
  common
  civmap_core
  cyber
  proto_msg
  )

  add_subdirectory(vtk_source)
  # add_executable(channel_test
  #   channel_test.cpp
  # )
  # target_link_libraries(channel_test
  # PUBLIC  
  # cyber
  # )