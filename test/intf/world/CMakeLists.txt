add_definitions(-DCURRENT_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(PETestWorld test.cpp)

target_link_libraries(PETestWorld PEngine PEUtils PEViewer)

#f 1 2 3 4
#f 5 6 7
#f 17 18 19 20 21
#f 22 23 24 25 26 27
#f 28 29 30 31
#f 32 33 34 35 36 37 38
#f 39 40 41 42
#f 43 44 45 46 47
#f 48 49 50 51 52
#f 53 54 55 56
#f 57 58 59 60
