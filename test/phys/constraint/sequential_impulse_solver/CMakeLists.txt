add_executable(PETestSISolver test.cpp ${PHYS_COLLISION_ALGO}
        ${ENGINE_INCLUDE_DIR}/phys/constraint/constraint_solver/sequential_impulse_solver.cpp
        ${ENGINE_INCLUDE_DIR}/phys/constraint/constraint/constraint.cpp
        ${ENGINE_INCLUDE_DIR}/phys/constraint/constraint/friction_contact_constraint.cpp
        ${ENGINE_INCLUDE_DIR}/phys/constraint/constraint/primal_dual/non_smooth_forces/force_constraint/force_constraint_base.cpp
        ${ENGINE_INCLUDE_DIR}/phys/constraint/constraint/primal_dual/non_smooth_forces/force_constraint/lorentz_circle_constraint.cpp
        ${ENGINE_INCLUDE_DIR}/phys/constraint/constraint/primal_dual/non_smooth_forces/non_smooth_force_base.cpp
        ${ENGINE_INCLUDE_DIR}/phys/constraint/constraint/primal_dual/non_smooth_forces/non_smooth_contact_force.cpp
        ${ENGINE_INCLUDE_DIR}/phys/object/rigidbody.cpp
        ${ENGINE_INCLUDE_DIR}/phys/collision/narrow_phase/contact_result.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/shape.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/box_shape.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/compound_shape.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/convex_mesh_shape.cpp
        ${ENGINE_INCLUDE_DIR}/phys/collision/narrow_phase/narrow_phase_base.cpp
        ${ENGINE_INCLUDE_DIR}/phys/collision/narrow_phase/simple_narrow_phase.cpp)

target_link_libraries(PETestSISolver PEUtils)
