cmake_minimum_required(VERSION 3.5)
project(lidar_node)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#_____________________________________
#Mariano
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)
include_directories(${YAMLCPP_INCLUDE_DIRS})

find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ground_plane_fit REQUIRED)

find_package(PCL 1.3 REQUIRED COMPONENTS common io conversions visualization filters features search sample_consensus segmentation)

include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(lidar_node src/lidar_node.cpp)
ament_target_dependencies(lidar_node
  "geometry_msgs"  # maybe don't need this
  "pcl_conversions"
  "rcl_interfaces"
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "std_msgs"
  "tf2_geometry_msgs"
  visualization_msgs
  ground_plane_fit
  yaml-cpp
)
target_link_libraries(lidar_node
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
  #Questo risolve il problema di undefined reference to pcl::console::print. Perchè? Ottima domanda.
  ${PCL_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  yaml-cpp
)

#Mariano
#_____________________________________

install(TARGETS lidar_node
        DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

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
