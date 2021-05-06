cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(keypoint_extraction)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (keypoint_extraction src/main.cpp src/iss_keypoints.cpp)
target_link_libraries (keypoint_extraction ${PCL_LIBRARIES})

add_executable (narf_keypoint_extraction src/narf_keypoint_extraction.cpp)
target_link_libraries (narf_keypoint_extraction ${PCL_LIBRARIES})

add_executable (iss_keypoint_extraction src/iss_keypoint_extraction.cpp)
target_link_libraries (iss_keypoint_extraction ${PCL_LIBRARIES})

add_executable (z_keypoint_extraction src/example_sift_z_keypoint_estimation.cpp)
target_link_libraries (z_keypoint_extraction ${PCL_LIBRARIES})

add_executable (normal_keypoint_extraction src/example_sift_normal_keypoint_estimation.cpp)
target_link_libraries (normal_keypoint_extraction ${PCL_LIBRARIES})
