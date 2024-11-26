cmake_minimum_required(VERSION 3.10)
project(PhysicsEngine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(TEST_ENABLED OFF)
set(ENABLE_EXTENDED_ALIGNED_STORAGE ON)

if (ENABLE_EXTENDED_ALIGNED_STORAGE)
    add_compile_definitions("_ENABLE_EXTENDED_ALIGNED_STORAGE")
else()
    add_compile_definitions("_DISABLE_EXTENDED_ALIGNED_STORAGE")
endif()

set(ENGINE_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/src)
set(EXTERN_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/ThirdParty)
set(SV_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/SimpleViewer/include)
set(SV_LIB_DIR ${CMAKE_SOURCE_DIR}/SimpleViewer/lib)
set(SV_BIN_DIR ${CMAKE_SOURCE_DIR}/SimpleViewer/bin)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

add_definitions(-DPE_USE_DOUBLE)
add_definitions(-DPE_MULTI_THREAD)
add_definitions(-DPE_DATA_PATH="${CMAKE_SOURCE_DIR}/data/")

include_directories(${ENGINE_INCLUDE_DIR})
include_directories(${EXTERN_INCLUDE_DIR})
include_directories(${SV_INCLUDE_DIR})

# Third party
add_definitions(-DPE_UTILS_USE_NFD)
add_subdirectory(${EXTERN_INCLUDE_DIR}/nfd)
include_directories(${EXTERN_INCLUDE_DIR}/nfd/include)
set_target_properties(nfd PROPERTIES LINKER_LANGUAGE CXX FOLDER ThirdParty)

add_subdirectory(src)
add_subdirectory(examples)
if(TEST_ENABLED)
    add_subdirectory(test)
endif()
