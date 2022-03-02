cmake_minimum_required(VERSION 3.5)

project(rmf_traffic_examples)

find_package(rmf_traffic REQUIRED)

add_executable(three_robots src/three_robots.cpp)
target_link_libraries(three_robots PUBLIC rmf_traffic::rmf_traffic)
