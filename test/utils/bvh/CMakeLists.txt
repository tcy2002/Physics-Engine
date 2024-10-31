add_definitions(-DBVH_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(PETestBVH test.cpp)

target_link_libraries(PETestBVH PEUtils PEViewer)