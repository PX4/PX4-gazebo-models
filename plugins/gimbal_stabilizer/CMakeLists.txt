cmake_minimum_required(VERSION 3.10)
project(GimbalStabilizerPlugin)

# Ensure the plugin loads system protobuf, not conda's conflicting version.
# CMAKE_BUILD_RPATH embeds /usr/lib/x86_64-linux-gnu into the .so, but on Ubuntu
# GNU ld defaults to --enable-new-dtags which stores it as DT_RUNPATH (ELF tag 0x1d).
# DT_RUNPATH has lower priority than LD_LIBRARY_PATH, so conda's protobuf still wins
# when the conda env is active. --disable-new-dtags forces DT_RPATH (tag 0x0f) which
# has higher priority than LD_LIBRARY_PATH, ensuring the system protobuf is loaded.
set(CMAKE_BUILD_RPATH "/usr/lib/x86_64-linux-gnu")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags")

find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-msgs10 REQUIRED)

add_library(${PROJECT_NAME} SHARED GimbalStabilizerSystem.cpp)

target_link_libraries(${PROJECT_NAME}
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
    gz-plugin2::gz-plugin2
    gz-msgs10::gz-msgs10
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)
