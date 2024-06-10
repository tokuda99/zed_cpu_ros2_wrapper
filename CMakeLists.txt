cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(zed_cpu_ros2_wrapper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -O2 -mtune=native -march=native -mfpmath=both -Wall -Wextra -Wno-long-long -pedantic"
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -idirafter /usr/include")
add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_definitions(-DVIDEO_MOD_AVAILABLE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights uncomment
  # the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE) the following line skips cpplint (only
  # works in a git repo) uncomment the line when this package is not in a git
  # repo set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_subdirectory(lib/zed-open-capture)

add_executable(zed_cpu_ros2_wrapper_node src/zed_cpu_ros2_wrapper_node.cpp
                                         src/zed_cpu_ros2_wrapper.cpp)
target_include_directories(
  zed_cpu_ros2_wrapper_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
target_link_libraries(zed_cpu_ros2_wrapper_node zed_open_capture)
ament_target_dependencies(zed_cpu_ros2_wrapper_node rclcpp sensor_msgs image_transport cv_bridge OpenCV camera_info_manager Boost)

install(TARGETS zed_cpu_ros2_wrapper_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()

# set(ROS_DISTRO $ENV{ROS_DISTRO}) if(${ROS_DISTRO} STREQUAL "rolling")
# add_compile_definitions(ROS_DISTRO_ROLLING) elseif(${ROS_DISTRO} STREQUAL
# "foxy") add_compile_definitions(ROS_DISTRO_FOXY) elseif(${ROS_DISTRO} STREQUAL
# "galactic") add_compile_definitions(ROS_DISTRO_GALACTIC) elseif(${ROS_DISTRO}
# STREQUAL "humble") add_compile_definitions(ROS_DISTRO_HUMBLE) endif()
