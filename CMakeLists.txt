cmake_minimum_required(VERSION 3.0.2)
project(pcl_registration_demo)

find_package(PCL REQUIRED)
# find_package(Boost REQUIRED COMPONENTS filesystem) find_package(Eigen3 QUIET)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(normal_distributions_transform
               normal_distributions_transform.cpp)
target_link_libraries(normal_distributions_transform ${PCL_LIBRARIES})

add_executable(interactive_icp_registration interactive_icp_registration.cpp)
target_link_libraries(interactive_icp_registration ${PCL_LIBRARIES})
