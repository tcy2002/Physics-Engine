cmake_minimum_required(VERSION 3.22)
project(PhysicsEngine)

set(CMAKE_CXX_STANDARD 17)

set(OPENGL_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/opengl/include)
set(OPENGL_LIB_DIR ${CMAKE_SOURCE_DIR}/opengl/lib/x64)
set(ENGINE_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/src)
set(ENGINE_CORE_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/src/core)

add_definitions(-DUSE_DOUBLE)

add_subdirectory(src)

add_subdirectory(test)
