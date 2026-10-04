# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target yolo_msgs::yolo_msgs
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${yolo_msgs_TARGETS}.
if(yolo_msgs_TARGETS AND NOT TARGET yolo_msgs::yolo_msgs)
  add_library(yolo_msgs::yolo_msgs INTERFACE IMPORTED)
  set_target_properties(yolo_msgs::yolo_msgs PROPERTIES
    INTERFACE_LINK_LIBRARIES "${yolo_msgs_TARGETS}")
endif()
