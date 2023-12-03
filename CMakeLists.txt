cmake_minimum_required(VERSION 3.22)
project(PhysicsEngine)

set(CMAKE_CXX_STANDARD 17)

# install static libraries to "/lib", dynamic libraries and executables to "/bin"
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE})

# install headers to "/include"
file(INSTALL ${CMAKE_SOURCE_DIR}/src/ DESTINATION ${CMAKE_SOURCE_DIR}/include FILES_MATCHING PATTERN "*.h" PATTERN "*.inl")

if(CMAKE_SIZEOF_VOID_P EQUAL 8) # 64-bit
    set(OPENGL_LIB_DIR ${CMAKE_SOURCE_DIR}/opengl/lib/x64)
else()
    set(OPENGL_LIB_DIR ${CMAKE_SOURCE_DIR}/opengl/lib)
endif()
set(OPENGL_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/opengl/include)
set(ENGINE_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/src)

add_definitions(-DPE_USE_DOUBLE)

add_subdirectory(src)
add_subdirectory(test)
