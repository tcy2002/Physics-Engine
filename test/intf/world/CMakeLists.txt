add_definitions(-DCURRENT_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(PETestWorld test.cpp)

target_link_libraries(PETestWorld PEngine PEUtils PEViewer)
