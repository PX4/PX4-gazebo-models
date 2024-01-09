cmake_minimum_required(VERSION 3.10)
project(px4_gz_plugins)

find_package(gz-cmake3 REQUIRED)
gz_find_package(gz-sim7 REQUIRED)

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)

# Add sources for each plugin to be registered.
add_library(topic_remap SHARED topicremap_plugin.cc )
set_property(TARGET topic_remap PROPERTY CXX_STANDARD 17)
target_link_libraries(topic_remap
  gz-sim${gz-sim7_VERSION_MAJOR}::gz-sim${gz-sim7_VERSION_MAJOR}
)
