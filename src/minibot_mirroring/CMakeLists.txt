cmake_minimum_required(VERSION 3.8)
project(minibot_mirroring)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)# uncomment the following section in order to fill in
find_package(telebot_interfaces REQUIRED)
find_package(node_utils REQUIRED)

add_executable(minibot_mirror src/minibot_mirror.cpp)
add_executable(torquer src/torquer.cpp)
# add_executable(torquer scripts/torquer.py)
ament_target_dependencies(torquer rclcpp telebot_interfaces node_utils)
ament_target_dependencies(minibot_mirror rclcpp telebot_interfaces node_utils)
install(TARGETS
  minibot_mirror
  torquer
  #<executable>
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
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

ament_package()
