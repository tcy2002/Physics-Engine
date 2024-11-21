add_definitions(-DTERRAIN_CAR_DEMO_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(PEDemoTerrainCar main.cpp)

target_link_libraries(PEDemoTerrainCar PEngine PEViewer PEUtils)
