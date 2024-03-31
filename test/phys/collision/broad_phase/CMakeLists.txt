add_executable(PETestBroadPhase test.cpp
        ${ENGINE_INCLUDE_DIR}/phys/collision/broad_phase/broad_phase_base.cpp
        ${ENGINE_INCLUDE_DIR}/phys/collision/broad_phase/broad_phase_sweep_and_prune.cpp
        ${ENGINE_INCLUDE_DIR}/phys/object/rigidbody.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/box_shape.cpp)

target_link_libraries(PETestBroadPhase PEUtils)
