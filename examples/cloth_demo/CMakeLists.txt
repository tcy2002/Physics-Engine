add_executable(PEDemoCloth main.cpp)

add_definitions(-DPE_DEMO_PATH="${CMAKE_CURRENT_SOURCE_DIR}/")

target_link_libraries(PEDemoCloth PEngine PEViewer PEUtils)
