cmake_minimum_required(VERSION 3.8)
project(nav2_rviz_plugins)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(backward_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_route REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

# Ensure yaml-cpp::yaml-cpp target exists
# yaml_cpp_vendor should provide this, but if not, create it from system yaml-cpp
if(NOT TARGET yaml-cpp::yaml-cpp)
  if(TARGET yaml-cpp)
    # If yaml-cpp target exists without namespace, create alias
    add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
  elseif(yaml-cpp_FOUND OR YAML_CPP_FOUND)
    # Try common variable names from FindYAML-CPP or yaml-cpp config
    set(YAML_CPP_LIB "")
    set(YAML_CPP_INCLUDE_DIR "")

    # Check various possible variable names
    if(DEFINED yaml-cpp_LIBRARIES)
      set(YAML_CPP_LIB "${yaml-cpp_LIBRARIES}")
    elseif(DEFINED YAML_CPP_LIBRARIES)
      set(YAML_CPP_LIB "${YAML_CPP_LIBRARIES}")
    elseif(DEFINED yaml-cpp_LIBRARY)
      set(YAML_CPP_LIB "${yaml-cpp_LIBRARY}")
    elseif(DEFINED YAML_CPP_LIBRARY)
      set(YAML_CPP_LIB "${YAML_CPP_LIBRARY}")
    endif()

    if(DEFINED yaml-cpp_INCLUDE_DIRS)
      set(YAML_CPP_INCLUDE_DIR "${yaml-cpp_INCLUDE_DIRS}")
    elseif(DEFINED YAML_CPP_INCLUDE_DIRS)
      set(YAML_CPP_INCLUDE_DIR "${YAML_CPP_INCLUDE_DIRS}")
    elseif(DEFINED yaml-cpp_INCLUDE_DIR)
      set(YAML_CPP_INCLUDE_DIR "${yaml-cpp_INCLUDE_DIR}")
    elseif(DEFINED YAML_CPP_INCLUDE_DIR)
      set(YAML_CPP_INCLUDE_DIR "${YAML_CPP_INCLUDE_DIR}")
    endif()

    # If we have the variables, create the target
    if(YAML_CPP_LIB AND YAML_CPP_INCLUDE_DIR)
      add_library(yaml-cpp::yaml-cpp UNKNOWN IMPORTED)
      set_target_properties(yaml-cpp::yaml-cpp PROPERTIES
        IMPORTED_LOCATION "${YAML_CPP_LIB}"
        INTERFACE_INCLUDE_DIRECTORIES "${YAML_CPP_INCLUDE_DIR}")
    else()
      # Fallback: try to find it manually
      find_library(YAML_CPP_LIBRARY_FOUND
        NAMES yaml-cpp libyaml-cpp
        PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
      )
      find_path(YAML_CPP_INCLUDE_DIR_FOUND
        NAMES yaml-cpp/yaml.h
        PATHS /usr/include /usr/local/include
      )
      if(YAML_CPP_LIBRARY_FOUND AND YAML_CPP_INCLUDE_DIR_FOUND)
        add_library(yaml-cpp::yaml-cpp UNKNOWN IMPORTED)
        set_target_properties(yaml-cpp::yaml-cpp PROPERTIES
          IMPORTED_LOCATION "${YAML_CPP_LIBRARY_FOUND}"
          INTERFACE_INCLUDE_DIRECTORIES "${YAML_CPP_INCLUDE_DIR_FOUND}")
      else()
        message(FATAL_ERROR "yaml-cpp::yaml-cpp target not found and could not be created")
      endif()
    endif()
  else()
    message(FATAL_ERROR "yaml-cpp package not found. Please install libyaml-cpp-dev")
  endif()
endif()

# In Kilted and older distros, rviz with Qt6 is not supported
if(TARGET Qt5::Core)
  set(QT_VERSION_MAJOR 5)
  set(QT_VERSION ${Qt5Core_VERSION})
  # In Qt5, state machine is part of core
  find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)
elseif(TARGET Qt6::Core)
  set(QT_VERSION_MAJOR 6)
  set(QT_VERSION ${Qt6Core_VERSION})
  find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets StateMachine Test Concurrent)
endif()
if(${QT_VERSION} VERSION_LESS 5.15.0)
  function(qt_wrap_cpp out)
    qt5_wrap_cpp(_sources ${ARGN})
    set("${out}" ${_sources} PARENT_SCOPE)
  endfunction()
  function(qt_wrap_ui out)
      qt5_wrap_ui(_uis_hdrs ${ARGN})
      set("${out}" ${_uis_hdrs} PARENT_SCOPE)
  endfunction()
endif()

set(QT_LIBRARIES
  Qt${QT_VERSION_MAJOR}::Core
  Qt${QT_VERSION_MAJOR}::Gui
  Qt${QT_VERSION_MAJOR}::Widgets
)
if(QT_VERSION_MAJOR EQUAL 6)
  list(APPEND QT_LIBRARIES Qt6::StateMachine)
endif()

nav2_package()

# We specifically don't turn on CMAKE_AUTOMOC, since it generates one huge
# mocs_compilation.cpp file that takes a lot of memory to compile.  Instead
# we create individual moc files that can be compiled separately.
set(nav2_rviz_plugins_headers_to_moc
  include/nav2_rviz_plugins/costmap_cost_tool.hpp
  include/nav2_rviz_plugins/docking_panel.hpp
  include/nav2_rviz_plugins/goal_pose_updater.hpp
  include/nav2_rviz_plugins/goal_tool.hpp
  include/nav2_rviz_plugins/nav2_panel.hpp
  include/nav2_rviz_plugins/route_tool.hpp
  include/nav2_rviz_plugins/selector.hpp
  include/nav2_rviz_plugins/particle_cloud_display/particle_cloud_display.hpp
)

foreach(header "${nav2_rviz_plugins_headers_to_moc}")
  qt_wrap_cpp(nav2_rviz_plugins_moc_files "${header}")
endforeach()

qt_wrap_ui(route_tool_UIS_H resource/route_tool.ui)

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/costmap_cost_tool.cpp
  src/docking_panel.cpp
  src/goal_tool.cpp
  src/nav2_panel.cpp
  src/route_tool.cpp
  src/selector.cpp
  src/utils.cpp
  src/particle_cloud_display/flat_weighted_arrows_array.cpp
  src/particle_cloud_display/particle_cloud_display.cpp
  ${nav2_rviz_plugins_moc_files}
  ${route_tool_UIS_H}
)
target_include_directories(${library_name} PUBLIC
  ${OGRE_INCLUDE_DIRS}
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${library_name} PUBLIC
  ${geometry_msgs_TARGETS}
  nav2_lifecycle_manager::nav2_lifecycle_manager_core
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  nav2_ros_common::nav2_ros_common
  nav2_route::route_server_core
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rviz_common::rviz_common
  rviz_default_plugins::rviz_default_plugins
  rviz_rendering::rviz_rendering
  ${std_msgs_TARGETS}
  tf2_geometry_msgs::tf2_geometry_msgs
  ${visualization_msgs_TARGETS}
  ${QT_LIBRARIES}
)
target_link_libraries(${library_name} PRIVATE
  ament_index_cpp::ament_index_cpp
  pluginlib::pluginlib
  yaml-cpp::yaml-cpp
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

install(DIRECTORY rviz launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  nav2_lifecycle_manager
  nav2_msgs
  nav2_route
  nav2_util
  Qt${QT_VERSION_MAJOR}
  rclcpp
  rclcpp_action
  rviz_common
  rviz_default_plugins
  rviz_rendering
  std_msgs
  tf2_geometry_msgs
  nav2_ros_common
  visualization_msgs
)

ament_package()
