cmake_minimum_required(VERSION 3.8)
project(b2z1_description)

# Enable good warnings if using GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install resources
install(
  DIRECTORY launch rviz urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
