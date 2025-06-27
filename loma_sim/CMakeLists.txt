cmake_minimum_required(VERSION 3.8)
project(loma_sim)

# Enable good warnings if using GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(raisim REQUIRED)
find_package(commutils REQUIRED)

# Get root path
get_filename_component(ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR} PATH)

# Enable testing
include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)
endif()

# Declare the shared library
add_library(${PROJECT_NAME}
  SHARED
    src/raisim_environment.cpp
)
# Specify the include directories
target_include_directories(${PROJECT_NAME}
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# Link to dependencies
target_link_libraries(${PROJECT_NAME} 
  PUBLIC
    Eigen3::Eigen
    raisim::raisim
    commutils::commutils
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install the target
install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Export targets for find_package
install(EXPORT export_${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)


# Export for ament
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})


ament_package()
