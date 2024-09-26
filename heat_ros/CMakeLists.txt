cmake_minimum_required(VERSION 3.5.1)

# Extract package name and version from package.xml
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)

project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

find_package(libgeodesic REQUIRED)
find_package(noether_gui REQUIRED)

# Library
add_library(${PROJECT_NAME} SHARED
  src/heat_method_tool_path_planner.cpp
  src/heat_method_tool_path_planner_widget.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  libgeodesic::libgeodesic
  libgeodesic::libgeodesic_cpp
  noether::noether_gui)
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 14)
target_clang_tidy(${PROJECT_NAME}
  ENABLE ${HEAT_ROS_ENABLE_CLANG_TIDY}
  WARNINGS_AS_ERRORS ${HEAT_ROS_ENABLE_TESTING}
  CHECKS ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(${PROJECT_NAME} INTERFACE ALL ENABLE ${HEAT_ROS_ENABLE_TESTING})

# Plugin library
add_library(${PROJECT_NAME}_plugins SHARED src/plugins.cpp)
target_link_libraries(${PROJECT_NAME}_plugins PUBLIC ${PROJECT_NAME})

# Install headers
install(DIRECTORY include/ DESTINATION include/)

# Package configuration
configure_package(
  NAMESPACE
    heat_ros
  DEPENDENCIES
    libgeodesic
    noether_gui
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_plugins
)
