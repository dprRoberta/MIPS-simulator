cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME MIPS_simulator)

project(${PROJECT_NAME})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -lm")

set(INCLUDE_DIRS ${INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)

add_executable(mipssim src/mipssim.c src/memory_hierarchy.c )

