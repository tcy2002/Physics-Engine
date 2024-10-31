add_definitions(-DCONCAVE_DEMO_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(PEDemoConcave main.cpp)

target_link_libraries(PEDemoConcave PEngine PEViewer PEUtils)
