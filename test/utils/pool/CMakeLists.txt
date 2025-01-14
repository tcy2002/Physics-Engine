add_executable(PETestPool test.cpp
        ${ENGINE_INCLUDE_DIR}/phys/constraint/constraint/constraint.cpp
        ${ENGINE_INCLUDE_DIR}/phys/constraint/constraint/friction_contact_constraint.cpp
        ${ENGINE_INCLUDE_DIR}/phys/constraint/constraint/primal_dual/non_smooth_forces/force_constraint/force_constraint_base.cpp
        ${ENGINE_INCLUDE_DIR}/phys/constraint/constraint/primal_dual/non_smooth_forces/force_constraint/lorentz_circle_constraint.cpp
        ${ENGINE_INCLUDE_DIR}/phys/constraint/constraint/primal_dual/non_smooth_forces/non_smooth_force_base.cpp
        ${ENGINE_INCLUDE_DIR}/phys/constraint/constraint/primal_dual/non_smooth_forces/non_smooth_contact_force.cpp
        ${ENGINE_INCLUDE_DIR}/phys/object/rigidbody.cpp
        ${ENGINE_INCLUDE_DIR}/phys/collision/narrow_phase/contact_result.cpp)

target_link_libraries(PETestPool PEUtils)