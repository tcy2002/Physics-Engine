add_executable(PETestPDSolver test.cpp ${PHYS_COLLISION_ALGO}
        ${ENGINE_INCLUDE_DIR}/rigid/constraint/constraint_solver/primal_dual_solver.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/constraint/constraint_solver/sequential_impulse_solver.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/constraint/constraint/primal_dual/non_smooth_forces/force_constraint/force_constraint_base.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/constraint/constraint/primal_dual/non_smooth_forces/force_constraint/lorentz_circle_constraint.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/constraint/constraint/primal_dual/non_smooth_forces/non_smooth_force_base.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/constraint/constraint/primal_dual/non_smooth_forces/non_smooth_contact_force.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/constraint/constraint/constraint.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/constraint/constraint/friction_contact_constraint.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/object/rigidbody.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/collision/narrow_phase/contact_result.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/box_shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/compound_shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/convex_mesh_shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/collision/narrow_phase/narrow_phase_base.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/collision/narrow_phase/simple_narrow_phase.cpp)

target_link_libraries(PETestPDSolver PEUtils)
