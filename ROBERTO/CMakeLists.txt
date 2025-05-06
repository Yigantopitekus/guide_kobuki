
cmake_minimum_required(VERSION 3.5)
project(guide_kobuki)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    behaviortree_cpp_v3
    ament_index_cpp
    nav2_msgs
    rclcpp_action
    kobuki_ros_interfaces
    geometry_msgs
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(navigate_to_objective SHARED src/library_lib/NavObjective.cpp)
add_library(search_node SHARED src/library_lib/Search.cpp)
add_library(navigate_to_odom SHARED src/library_lib/NavOdom.cpp)
add_library(store_object_node SHARED src/library_lib/StoreObject.cpp)
add_library(readqr_node SHARED src/library_lib/ReadQR.cpp)

list(APPEND plugin_libs
  navigate_to_objective
  search_node
  navigate_to_odom
  store_object_node
  readqr_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(guide_nav_main src/guide_nav_main.cpp)
ament_target_dependencies(guide_nav_main ${dependencies})


install(TARGETS
  ${plugin_libs}
  guide_nav_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  bt_xml
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
