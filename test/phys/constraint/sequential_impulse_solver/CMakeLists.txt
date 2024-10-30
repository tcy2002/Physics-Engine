add_executable(PETestSISolver test.cpp ${PHYS_COLLISION_ALGO}
        ${ENGINE_INCLUDE_DIR}/phys/constraint/constraint_solver/sequential_impulse_constraint_solver.cpp
        ${ENGINE_INCLUDE_DIR}/phys/constraint/constraint/friction_contact_constraint.cpp
        ${ENGINE_INCLUDE_DIR}/phys/object/rigidbody.cpp
        ${ENGINE_INCLUDE_DIR}/phys/collision/narrow_phase/contact_result.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/box_shape.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/compound_shape.cpp
        ${ENGINE_INCLUDE_DIR}/phys/collision/narrow_phase/simple_narrow_phase.cpp)

target_link_libraries(PETestSISolver PEUtils)
