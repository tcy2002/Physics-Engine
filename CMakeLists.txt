cmake_minimum_required(VERSION 3.10)
project(PhysicsEngine)

set(CMAKE_CXX_STANDARD 14)

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi /EHsc /MP /bigobj")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Zi /Od")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2 /Ob2 /Oi /Ot /GL")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wno-unused-parameter -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-function -Wno-unused-result")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
endif()

set(ENGINE_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/src)
set(SV_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/SimpleViewer/include)
set(SV_LIB_DIR ${CMAKE_SOURCE_DIR}/SimpleViewer/lib)
set(SV_BIN_DIR ${CMAKE_SOURCE_DIR}/SimpleViewer/bin)

# install SimpleViewer dlls to "/bin"
file(INSTALL
    ${SV_BIN_DIR}/${CMAKE_BUILD_TYPE}/ ${SV_BIN_DIR}/freeglut.dll ${SV_BIN_DIR}/glew32.dll
    DESTINATION ${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE}/
    FILES_MATCHING PATTERN "*.dll")

# install static libraries to "/lib", dynamic libraries and executables to "/bin"
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE})

## install headers to "/include"
#file(INSTALL
#    ${CMAKE_SOURCE_DIR}/src/
#    DESTINATION ${CMAKE_SOURCE_DIR}/include
#    FILES_MATCHING PATTERN "*.h" PATTERN "*.inl")

add_definitions(-DPE_USE_DOUBLE)

add_subdirectory(src)
add_subdirectory(test)
