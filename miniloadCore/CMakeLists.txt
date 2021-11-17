cmake_minimum_required(VERSION 3.5)
project(miniloadcore)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

## include directories
#set(THIS_PACKAGE_INCLUDE_DIRS
#        common/include
#        communication/include
#        motor/include)
#
#set(THIS_PACKAGE_LIBRARIES
#        communication/lib
#)
#
#include_directories(
#        ${THIS_PACKAGE_INCLUDE_DIRS}
#        ${rclcpp_INCLUDE_DIRS}
#)

add_subdirectory(common)
add_subdirectory(communication)
add_subdirectory(motor)
add_subdirectory(examples)

#link_directories(
#        lib
#        ${rclcpp_LIB_DIRS}
#)


######################
########INSTALL#######
######################

#install(TARGETS
#        can_drive_pkg
#        main
#        DESTINATION lib/${PROJECT_NAME}
#)


#install(
#        DIRECTORY ${THIS_PACKAGE_LIBRARIES}
#        DESTINATION lib/${PROJECT_NAME}
#)
#
#


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
