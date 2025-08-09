add_executable(PETestNarrowPhase test.cpp ${PHYS_COLLISION_ALGO}
        ${ENGINE_INCLUDE_DIR}/rigid/collision/narrow_phase/contact_result.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/collision/narrow_phase/narrow_phase_base.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/collision/narrow_phase/simple_narrow_phase.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/object/rigidbody.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/box_shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/convex_mesh_shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/compound_shape.cpp)

target_link_libraries(PETestNarrowPhase PEUtils)
