cmake_minimum_required(VERSION 3.8)
project(roboclaw_sdk)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(ROBOCLAW_SOURCES
  src/driver.cpp
  src/TimeoutSerial.cpp
)

#Create library for our include files
add_library(roboclaw_sdk SHARED ${ROBOCLAW_SOURCES}) #specify only the cpp files in here, can be multiple
# this line to exports the library
ament_export_targets(roboclaw_sdk HAS_LIBRARY_TARGET)

#Allows executable to be linked properly
install(
  DIRECTORY include/roboclaw_sdk
  DESTINATION include
)
#Actually move library to install
install(
  TARGETS roboclaw_sdk
  EXPORT roboclaw_sdk
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)
#This handles the linking of the library
target_include_directories(roboclaw_sdk #Library name, in this case the package name
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

#Declare dependencies
# NONE

#Not really sure why this is needed but it is, just add ur executables into the list
install(TARGETS
  #<executable>
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#This links the multiplexer executable to the header files contained in our package
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