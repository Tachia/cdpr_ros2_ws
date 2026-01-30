cmake_minimum_required(VERSION 3.10)
project(cdpr_gz_plugins)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)

add_library(cdpr_cable_plugin SHARED
  src/cdpr_cable_plugin.cpp
)

target_link_libraries(cdpr_cable_plugin
  gz-sim8::gz-sim8
)

install(TARGETS cdpr_cable_plugin
  LIBRARY DESTINATION lib
)

ament_package()

