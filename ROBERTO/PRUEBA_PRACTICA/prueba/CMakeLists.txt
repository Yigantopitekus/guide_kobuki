cmake_minimum_required(VERSION 3.8)
project(prueba)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(image_geometry REQUIRED)
find_package(nav2_msgs REQUIRED)

set(dependencies
  ament_cmake 
  rclcpp
  behaviortree_cpp_v3 
  geometry_msgs 
  ament_index_cpp 
  kobuki_ros_interfaces 
  vision_msgs 
  image_transport 
  cv_bridge 
  message_filters 
  pcl_ros 
  pcl_conversions 
  image_geometry
  nav2_msgs

)

include_directories(include)

add_library(move_bt_node SHARED src/prueba/Move.cpp)
add_library(getwp_bt_node SHARED src/prueba/GetWaypoint.cpp)
add_library(show_list_node SHARED src/prueba/ShowList.cpp)
add_library(button_1_node SHARED src/prueba/B1Pressed.cpp)
add_library(button_0_node SHARED src/prueba/B2Pressed.cpp)
add_library(button_2_node SHARED src/prueba/B0Pressed.cpp)

list(APPEND plugin_libs
  move_bt_node
  getwp_bt_node
  show_list_node
  button_0_node
  button_1_node
  button_2_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(prueba src/main.cpp)
ament_target_dependencies(prueba ${dependencies})
target_link_libraries(prueba ${ZMQ_LIBRARIES})

install(TARGETS
  ${plugin_libs}
  prueba
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  behaviorTree
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
