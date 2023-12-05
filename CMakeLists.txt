cmake_minimum_required(VERSION 3.22)
project(PhysicsEngine)

set(CMAKE_CXX_STANDARD 17)
set(ENGINE_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/src)

# generate static libraries that could used by msvc
if (MSVC)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    set(BUILD_SHARED_LIBS OFF)
endif ()

# install static libraries to "/lib", dynamic libraries and executables to "/bin"
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE})

# install headers to "/include"
file(INSTALL
    ${CMAKE_SOURCE_DIR}/src/
    DESTINATION ${CMAKE_SOURCE_DIR}/include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.inl")

add_definitions(-DPE_USE_DOUBLE)

add_subdirectory(src)
add_subdirectory(test)
