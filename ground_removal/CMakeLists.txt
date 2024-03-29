cmake_minimum_required(VERSION 3.5)
project(ground_plane_fit)

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

find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)

add_library (ground_plane_fit ./src/ground_plane_fitting.cpp)
# this line to exports the library
ament_export_targets(ground_plane_fit HAS_LIBRARY_TARGET)

target_link_libraries (ground_plane_fit
    ${PCL_LIBRARIES}
)

ament_target_dependencies(ground_plane_fit "rclcpp")

target_include_directories(ground_plane_fit PRIVATE
        include)

ament_export_include_directories(include)

#Install library

install(
    FILES ./include/ground_plane_fitting.h
    DESTINATION include
)

install(
  TARGETS ground_plane_fit
  EXPORT ground_plane_fit
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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
