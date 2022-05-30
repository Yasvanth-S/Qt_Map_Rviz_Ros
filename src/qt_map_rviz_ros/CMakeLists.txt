cmake_minimum_required(VERSION 3.16.0)
project(qt_map_rviz_ros)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set (TARGET_NAME qt_map_rviz_ros)
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rviz
  std_msgs
)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets Multimedia Network)
set(QT_LIBRARIES Qt5::Widgets Qt5::Multimedia Qt5::Core Qt5::Network Qt5::Gui)
set(QTVERSION ${Qt5Widgets_VERSION})
include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/qt_map_rviz_ros/*.hpp *.h)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})


file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
add_executable(${TARGET_NAME} ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(${TARGET_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

