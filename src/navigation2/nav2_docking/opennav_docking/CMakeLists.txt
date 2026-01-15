cmake_minimum_required(VERSION 3.8)
project(opennav_docking)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(backward_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_graceful_controller REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(opennav_docking_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
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

find_package(nav2_ros_common REQUIRED)
find_package(bond REQUIRED)

nav2_package()

# Get workspace nav2_util include directories to ensure workspace version is used
get_target_property(nav2_util_include_dirs nav2_util::nav2_util_core INTERFACE_INCLUDE_DIRECTORIES)

set(executable_name opennav_docking)
set(library_name ${executable_name}_core)

add_library(controller SHARED
  src/controller.cpp
)
target_include_directories(controller
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(controller PUBLIC
  ${geometry_msgs_TARGETS}
  nav2_graceful_controller::nav2_graceful_controller
  nav2_ros_common::nav2_ros_common
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
)

add_library(${library_name} SHARED
  src/docking_server.cpp
  src/dock_database.cpp
  src/navigator.cpp
)
target_include_directories(${library_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
# Ensure workspace nav2_util header is found before system version
if(nav2_util_include_dirs)
  target_include_directories(${library_name} PRIVATE
    $<BUILD_INTERFACE:${nav2_util_include_dirs}>
  )
endif()
target_link_libraries(${library_name} PUBLIC
  angles::angles
  controller
  ${geometry_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  nav2_ros_common::nav2_ros_common
  opennav_docking_core::opennav_docking_core
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
  tf2_ros::tf2_ros
  yaml-cpp::yaml-cpp
)
target_link_libraries(${library_name} PRIVATE
  rclcpp_components::component
  tf2_geometry_msgs::tf2_geometry_msgs
)

add_library(pose_filter SHARED
  src/pose_filter.cpp
)
target_include_directories(pose_filter
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(pose_filter PUBLIC
  ${geometry_msgs_TARGETS}
  nav2_ros_common::nav2_ros_common
)
target_link_libraries(pose_filter PRIVATE
  rclcpp::rclcpp
  tf2_geometry_msgs::tf2_geometry_msgs
)

add_executable(${executable_name}
  src/main.cpp
)
target_include_directories(${executable_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${executable_name} PRIVATE ${library_name}
  rclcpp::rclcpp
  nav2_ros_common::nav2_ros_common
)

rclcpp_components_register_nodes(${library_name} "opennav_docking::DockingServer")

add_library(simple_charging_dock SHARED
  src/simple_charging_dock.cpp
)
target_include_directories(simple_charging_dock
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(simple_charging_dock PUBLIC
  ${geometry_msgs_TARGETS}
  ${bond_TARGETS}
  opennav_docking_core::opennav_docking_core
  pose_filter
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  nav2_ros_common::nav2_ros_common
  bondcpp::bondcpp
)
target_link_libraries(simple_charging_dock PRIVATE
  angles::angles
  nav2_util::nav2_util_core
  pluginlib::pluginlib
  tf2::tf2
)

add_library(simple_non_charging_dock SHARED
  src/simple_non_charging_dock.cpp
)
target_include_directories(simple_non_charging_dock
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(simple_non_charging_dock PUBLIC
  ${geometry_msgs_TARGETS}
  opennav_docking_core::opennav_docking_core
  nav2_ros_common::nav2_ros_common
  pose_filter
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
)
target_link_libraries(simple_non_charging_dock PRIVATE
  angles::angles
  nav2_util::nav2_util_core
  pluginlib::pluginlib
  tf2::tf2
)

pluginlib_export_plugin_description_file(opennav_docking_core plugins.xml)

install(TARGETS ${library_name} controller pose_filter simple_charging_dock simple_non_charging_dock
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY test/dock_files
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  ament_lint_auto_find_test_dependencies()
  ament_find_gtest()
  add_subdirectory(test)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name} controller pose_filter)
ament_export_dependencies(
  geometry_msgs
  nav2_graceful_controller
  nav2_msgs
  nav2_util
  opennav_docking_core
  pluginlib
  nav2_ros_common
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rcl_interfaces
  std_srvs
  tf2_ros
  yaml-cpp
)
ament_export_targets(${PROJECT_NAME})
ament_package()
