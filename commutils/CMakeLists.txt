cmake_minimum_required(VERSION 3.5)
project(commutils)

# Enable good warnings if using GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Enable testing
include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)
endif()

# Declare the interface library
add_library(${PROJECT_NAME} INTERFACE)
# Specify the include directories
target_include_directories(${PROJECT_NAME}
  INTERFACE 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# Link to dependencies
target_link_libraries(${PROJECT_NAME} 
  INTERFACE
    Eigen3::Eigen
    yaml-cpp
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

# Unit tests
if(BUILD_TESTING)
    add_executable(test_${PROJECT_NAME}
        test/gtest_main.cpp
        test/csv/test_csv_writer.cpp
        test/yaml/test_yaml_tools.cpp
        test/yaml/test_yaml_eigen.cpp
    )
    target_include_directories(test_${PROJECT_NAME}
      PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_link_libraries(test_${PROJECT_NAME}
      PRIVATE
        Eigen3::Eigen
        yaml-cpp
        GTest::gtest
    )
    gtest_discover_tests(test_${PROJECT_NAME})
endif()

# Export for ament
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
