cmake_minimum_required(VERSION 3.8)
project(m4_tracked_object_merger VERSION 1.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wconversion)
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()


# find dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(tracked_object_merger_node SHARED
  src/tracked_object_merger.cpp
  src/tracked_object_merger_utils.cpp
)

target_link_libraries(tracked_object_merger_node
  Eigen3::Eigen
)

rclcpp_components_register_node(tracked_object_merger_node
  PLUGIN "tracked_object_merger::TrackedObjectMergerNode"
  EXECUTABLE m4_tracked_object_merger
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
