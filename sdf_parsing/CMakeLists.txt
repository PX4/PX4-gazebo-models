cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(sdformat14 REQUIRED) # sdformat7 or higher

include_directories(${SDFormat_INCLUDE_DIRS})
link_directories(${SDFormat_LIBRARY_DIRS})

add_executable(check_sdf check_sdf.cc)
target_link_libraries(check_sdf ${SDFormat_LIBRARIES})