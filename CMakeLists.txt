cmake_minimum_required(VERSION 3.0.2)
project(slambox_ros)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++17 -O3)
set(BUILD_SHARED_LIBS OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(glog 0.6.0 REQUIRED)
find_package(ZLIB REQUIRED)
find_package(yaml-cpp REQUIRED )
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(slambox_sdk 0.2.0 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
include_directories(
  ${YAML_CPP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB SOURCES_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

# Remove executable cpp files
FOREACH(source_executable ${SOURCES_EXECUTABLE})
    list(REMOVE_ITEM SOURCES ${source_executable})
ENDFOREACH()

# Add executable list of executable cpp files
FOREACH(source_executable ${SOURCES_EXECUTABLE})
  get_filename_component(source_executable_name ${source_executable} NAME_WLE)
  set(executable_name ${PROJECT_NAME}-${source_executable_name})

  add_executable( ${executable_name}
      ${SOURCES}
      ${source_executable}
  )

  target_link_libraries(${executable_name}
    ${catkin_LIBRARIES}
    glog::glog
    ${slambox_sdk_LIBRARIES}
    ZLIB::ZLIB
    fmt::fmt
    ${YAML_CPP_LIBRARIES}
    ${PCL_LIBRARIES}
  )
ENDFOREACH()

add_subdirectory("libs/fmt")

# Unit test
if (CATKIN_ENABLE_TESTING)
    set(TEST_NAME_BASE test-${PROJECT_NAME})

    enable_testing()

    include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

    # Auto adding unit test files starting with test_*.cpp
    FILE(GLOB UNITTEST_LIST CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_*.cpp")
    FOREACH(unittest_path ${UNITTEST_LIST})
        get_filename_component(unittest_name ${unittest_path} NAME_WLE)
        string(REPLACE "test_" "" unittest_name ${unittest_name})
        set(TEST_NAME ${TEST_NAME_BASE}-${unittest_name})

        message("Build Unit Test for ${unittest_path} with name of ${TEST_NAME}")

        catkin_add_gtest(${TEST_NAME} ${unittest_path} ${SOURCES})

        target_link_libraries(${TEST_NAME}
            ${catkin_LIBRARIES}
            gtest
            gtest_main
            glog::glog
            ${slambox_sdk_LIBRARIES}
            fmt::fmt
            ${YAML_CPP_LIBRARIES}
            ${PCL_LIBRARIES}
        )

        add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    ENDFOREACH()
endif()
