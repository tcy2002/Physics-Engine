add_executable(PETestWorld test.cpp)

target_link_libraries(PETestWorld ${TEST_EXT_LIBS} PEViewer PEngine)
