add_executable(PETestWorld test.cpp
        ${ENGINE_INCLUDE_DIR}/core/world.cpp
        ${ENGINE_INCLUDE_DIR}/phys/constraint/solver/sequential_impulse_solver.cpp
        ${ENGINE_INCLUDE_DIR}/phys/constraint/constraint/friction_contact_constraint.cpp
        ${ENGINE_INCLUDE_DIR}/phys/collision/collision_algorithm/box_box_collision_algorithm.cpp
        ${ENGINE_INCLUDE_DIR}/phys/collision/narrow_phase/simple_narrow_phase.cpp
        ${ENGINE_INCLUDE_DIR}/phys/collision/narrow_phase/contact_result.cpp
        ${ENGINE_INCLUDE_DIR}/phys/collision/broad_phase/broad_phase_sweep_and_prune.cpp
        ${ENGINE_INCLUDE_DIR}/phys/object/rigidbody.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/box_shape.cpp)

target_link_libraries(PETestWorld ${TEST_EXT_LIBS} PEViewer)
