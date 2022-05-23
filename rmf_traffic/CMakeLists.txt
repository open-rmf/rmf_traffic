cmake_minimum_required(VERSION 3.5.0)

project(rmf_traffic VERSION 2.0.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  # Use the Release build type by default if the user has not specified one
  set(CMAKE_BUILD_TYPE Release)
endif()

include(GNUInstallDirs)

# We need to use the internal version of FCL because the upstream version has
# bugs that are not yet fixed.
#find_package(fcl 0.6 QUIET)

if(fcl_FOUND)
  message(STATUS "Using installed FCL version: ${FCL_VERSION}")
else()
  message(STATUS "Using in-source FCL")
  add_subdirectory(thirdparty/fcl)
endif()
# TODO(MXG): Remove everything related to maintaining compatibility with 0.5,
# since we are now requiring FCL-0.6+
set(using_new_fcl true)
set(FCL_LIBRARIES fcl)

find_package(rmf_utils REQUIRED)
find_package(Threads)

find_package(eigen3_cmake_module QUIET)
if (eigen3_cmake_module_FOUND)
  set(RMF_TRAFFIC_DEPENDS_EIGEN3_CMAKE_MODULE "find_dependency(eigen3_cmake_module)")
endif()
find_package(Eigen3 REQUIRED)

# ===== Traffic control library
file(GLOB_RECURSE core_lib_srcs "src/rmf_traffic/*.cpp")
add_library(rmf_traffic SHARED
  ${core_lib_srcs}
)

find_package(ament_cmake_catch2 QUIET)
find_package(ament_cmake_uncrustify QUIET)
if(BUILD_TESTING AND ament_cmake_catch2_FOUND AND ament_cmake_uncrustify_FOUND)
  file(GLOB_RECURSE unit_test_srcs "test/*.cpp")

  ament_add_catch2(
    test_rmf_traffic test/main.cpp ${unit_test_srcs}
    TIMEOUT 300)
  target_link_libraries(test_rmf_traffic
      rmf_traffic
      ${FCL_LIBRARIES}
      Threads::Threads
  )

  if(using_new_fcl)
    target_compile_definitions(test_rmf_traffic PRIVATE RMF_TRAFFIC__USING_FCL_0_6)
  endif()

  target_include_directories(test_rmf_traffic
    PUBLIC
      "${Eigen3_INCLUDE_DIRS}"
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/>
  )

  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../share/rmf_utils/")

  # Uncrustify is bugging out, giving nonsense false positives for the file
  # rmf_traffic/src/rmf_traffic/agv/Interpolate.cpp which have no apparent
  # solution. We will disable uncrustify until this big is fixed or a workaround
  # can be found.
#  ament_uncrustify(
#    ARGN include src test
#    CONFIG_FILE ${uncrustify_config_file}
#    MAX_LINE_LENGTH 80
#  )
endif()

if(using_new_fcl)
  target_compile_definitions(rmf_traffic PRIVATE RMF_TRAFFIC__USING_FCL_0_6)
endif()

target_link_libraries(rmf_traffic
  PUBLIC
    rmf_utils::rmf_utils
    Threads::Threads
  PRIVATE
    ${FCL_LIBRARIES}
)

target_include_directories(rmf_traffic
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${EIGEN3_INCLUDE_DIRS}
)

# Create cmake config files
include(CMakePackageConfigHelpers)

set(INSTALL_CONFIG_DIR "${CMAKE_INSTALL_LIBDIR}/rmf_traffic/cmake")
set(PACKAGE_CONFIG_VERSION_FILE "${CMAKE_CURRENT_BINARY_DIR}/rmf_traffic-config-version.cmake")
set(PACKAGE_CONFIG_FILE "${CMAKE_CURRENT_BINARY_DIR}/rmf_traffic-config.cmake")

configure_package_config_file(
  "${CMAKE_CURRENT_LIST_DIR}/cmake/rmf_traffic-config.cmake.in"
  ${PACKAGE_CONFIG_FILE}
  INSTALL_DESTINATION ${INSTALL_CONFIG_DIR}
)

write_basic_package_version_file(
  ${PACKAGE_CONFIG_VERSION_FILE}
  COMPATIBILITY SameMajorVersion
)

install(
  TARGETS rmf_traffic
  EXPORT  rmf_traffic-targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  DIRECTORY include/rmf_traffic
  DESTINATION include
)

install(
  FILES
    ${PACKAGE_CONFIG_VERSION_FILE}
    ${PACKAGE_CONFIG_FILE}
  DESTINATION ${INSTALL_CONFIG_DIR}
)

install(
  EXPORT rmf_traffic-targets
  FILE rmf_traffic-targets.cmake
  NAMESPACE rmf_traffic::
  DESTINATION ${INSTALL_CONFIG_DIR}
)

export(
  EXPORT rmf_traffic-targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/rmf_traffic-targets.cmake
  NAMESPACE rmf_traffic::
)

# Export ament index path
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv "prepend-non-duplicate;AMENT_PREFIX_PATH;")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv DESTINATION share/${PROJECT_NAME}/hook)
