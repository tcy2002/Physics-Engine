add_library(
    Core
    simulator.h simulator.cpp
    world.h world.cpp
)

target_link_libraries(Core PRIVATE Ui)
