cmake_minimum_required(VERSION 3.3.0)
project(fast_tf)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wpedantic -Wall -Wextra)
set(CATKIN_REQUIRED_PACKAGES geometry_msgs tf2 tf2_ros tf2_eigen tf2_msgs)
find_package(catkin REQUIRED ${CATKIN_REQUIRED_PACKAGES})
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS src
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS ${CATKIN_REQUIRED_PACKAGES}
)

###########
## Build ##
###########

add_library(${PROJECT_NAME} src/fast_tf.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC src ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME})

add_library(${PROJECT_NAME}_ros src/fast_tf_ros.cpp)
target_include_directories(${PROJECT_NAME}_ros PUBLIC src ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME} ${catkin_LIBRARIES})

####################
## Compile Checks ##
####################

# optional checks for development. follow the installation steps of iwyu to
# enable this feature. see https://github.com/include-what-you-use/include-what-you-use
# for more info.
find_program(iwyu_path NAMES include-what-you-use iwyu)
if(iwyu_path)
  message("running with include-what-you-get")
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path})
  set_property(TARGET ${PROJECT_NAME}_ros PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path})
endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY src/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(${PROJECT_NAME}_gtest
    test/test_dynamic_transform.cpp
    test/test_transform_buffer.cpp
    test/test_transform_tree.cpp)
  target_link_libraries(${PROJECT_NAME}_gtest ${PROJECT_NAME} gtest_main ${catkin_LIBRARIES})
  target_include_directories(${PROJECT_NAME}_gtest PRIVATE ${catkin_INCLUDE_DIRS})
endif()

###############
## Benchmark ##
###############

set(${PROJECT_NAME}_BENCHMARK ON)
if(${PROJECT_NAME}_BENCHMARK)
  # you may install the benchmark library via sudo apt install libbenchmark-dev
  find_package(benchmark)
  if(benchmark_FOUND)
    message("building benchmark target")
    add_executable(${PROJECT_NAME}_perf
      perf/perf_dynamic_transform.cpp
      perf/perf_transform_buffer.cpp
      perf/perf_transform_prod.cpp)
    target_link_libraries(${PROJECT_NAME}_perf benchmark::benchmark ${PROJECT_NAME} ${catkin_LIBRARIES})
    target_include_directories(${PROJECT_NAME}_perf PRIVATE ${catkin_INCLUDE_DIRS})
  else()
    message("benchmark library not found")
  endif()

  add_executable(listener_legacy perf/listener_legacy.cpp)
  target_include_directories(listener_legacy PUBLIC ${catkin_INCLUDE_DIRS})
  target_link_libraries(listener_legacy  ${catkin_LIBRARIES})

  add_executable(listener_new perf/listener_new.cpp)
  target_include_directories(listener_new PUBLIC ${PROJECT_NAME}_ros)
  target_link_libraries(listener_new ${PROJECT_NAME}_ros)
endif()
