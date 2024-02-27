add_executable(PETestNarrowPhase test.cpp
        ${ENGINE_INCLUDE_DIR}/phys/collision/collision_algorithm/box_box_collision_algorithm.cpp
        ${ENGINE_INCLUDE_DIR}/phys/collision/narrow_phase/simple_narrow_phase.cpp
        ${ENGINE_INCLUDE_DIR}/phys/collision/narrow_phase/contact_result.cpp
        ${ENGINE_INCLUDE_DIR}/phys/object/rigidbody.cpp
        ${ENGINE_INCLUDE_DIR}/phys/object/collision_object.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/box_shape.cpp)

target_link_libraries(PETestNarrowPhase ${TEST_EXT_LIBS})
