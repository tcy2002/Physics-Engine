add_executable(PETestBoxBoxCollision test.cpp
        ${ENGINE_INCLUDE_DIR}/phys/collision/collision_algorithm/box_box_collision_algorithm.cpp
        ${ENGINE_INCLUDE_DIR}/phys/object/rigidbody.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/shape.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/box_shape.cpp
        ${ENGINE_INCLUDE_DIR}/phys/collision/narrow_phase/contact_result.cpp)

target_link_libraries(PETestBoxBoxCollision PEUtils)
