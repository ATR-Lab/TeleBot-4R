cmake_minimum_required(VERSION 3.8)
project(node_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(telebot_interfaces REQUIRED)
set(SOURCES
  src/node_utils.cpp
)
add_library(node_utils SHARED ${SOURCES})#specify only the cpp files in here, can be multiple
# this line to exports the library
ament_export_targets(node_utils HAS_LIBRARY_TARGET)
ament_target_dependencies(node_utils telebot_interfaces rclcpp)

install(
  DIRECTORY include/node_utils
  DESTINATION include
)
#Actually move library to install
install(
  TARGETS node_utils
  EXPORT node_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY
  
  DESTINATION share/${PROJECT_NAME}
)
#This handles the linking of the library
target_include_directories(node_utils #Library name, in this case the package name
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
ament_export_dependencies(
  rclcpp
)
#ament_export_libraries(${PROJECT_NAME})
# ament_export_targets("export_${PROJECT_NAME}")

ament_package()
