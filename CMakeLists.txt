cmake_minimum_required(VERSION 3.5)
project(hri_rviz)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

add_definitions(-D_BUILD_DIR_PATH="${CMAKE_CURRENT_BINARY_DIR}")
add_definitions(-D_SRC_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(hri_msgs REQUIRED)
find_package(hri REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(pluginlib REQUIRED)

set(dependencies
  rclcpp
  rviz_common
  rviz_default_plugins
  rviz_rendering
  hri_msgs
  hri
  sensor_msgs
  cv_bridge
  image_transport
  pluginlib
)

set(hri_rviz_headers_to_moc
  include/hri_rviz/hri_humans.hpp
  # include/hri_rviz/hri_skeletons.hpp
  # include/hri_rviz/hri_tf.hpp
)

foreach(header "${hri_rviz_headers_to_moc}")
  qt5_wrap_cpp(hri_rviz_mocs "${header}")
endforeach()

set(hri_rviz_source_files
  src/hri_rviz/hri_humans.cpp
  src/hri_rviz/get_transport_from_topic.cpp
  src/hri_rviz/hri_skeletons.cpp
  # src/hri_rviz/hri_tf
)

add_library(hri_rviz SHARED
  ${hri_rviz_mocs}
  ${hri_rviz_source_files}
)
target_include_directories(hri_rviz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
)
target_compile_definitions(${PROJECT_NAME} PRIVATE 
"HRI_RVIZ_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)


ament_target_dependencies(hri_rviz
PUBLIC
${dependencies})

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(hri_rviz HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

install(
  TARGETS hri_rviz
  EXPORT hri_rviz
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

ament_package()