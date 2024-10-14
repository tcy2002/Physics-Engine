cmake_minimum_required(VERSION 3.10)
project(PhysicsEngine)

set(CMAKE_CXX_STANDARD 14)
set(TEST_ENABLED ON)

set(ENGINE_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/src)
set(SV_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/SimpleViewer/include)
set(SV_LIB_DIR ${CMAKE_SOURCE_DIR}/SimpleViewer/lib)
set(SV_BIN_DIR ${CMAKE_SOURCE_DIR}/SimpleViewer/bin)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

add_definitions(-DPE_USE_DOUBLE)
#add_definitions(-DPE_MULTI_THREAD)

include_directories(${ENGINE_INCLUDE_DIR})
include_directories(${SV_INCLUDE_DIR})

add_subdirectory(src)
add_subdirectory(examples)
if(TEST_ENABLED)
    add_subdirectory(test)
endif()
