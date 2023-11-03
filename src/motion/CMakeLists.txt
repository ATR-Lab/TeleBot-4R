cmake_minimum_required(VERSION 3.8)
project(motion)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(telebot_interfaces REQUIRED)

#Create library for our include files
add_library(motion SHARED src/control_source_handler.cpp)#specify only the cpp files in here, can be multiple
#This handles the linking of the library
target_include_directories(motion #Library name, in this case the package name
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

#Create an executable
add_executable(multiplexer src/multiplexer.cpp)
#Declare dependencies
ament_target_dependencies(multiplexer rclcpp std_msgs telebot_interfaces)
#Not really sure why this is needed but it is, just add ur executables into the list
install(TARGETS
  multiplexer
  #<executable>
  DESTINATION lib/${PROJECT_NAME})

#This links the multiplexer executable to the header files contained in our package
target_link_libraries(multiplexer motion)

#Idk what this does either
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

#This finalizes the build
ament_package()