add_executable(PETestBroadPhase test.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/collision/broad_phase/broad_phase_base.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/collision/broad_phase/broad_phase_sweep_and_prune.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/object/rigidbody.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/box_shape.cpp)

target_link_libraries(PETestBroadPhase PEUtils)
