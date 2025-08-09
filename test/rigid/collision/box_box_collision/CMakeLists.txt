add_executable(PETestBoxBoxCollision test.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/collision/collision_algorithm/box_box_collision_algorithm.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/object/rigidbody.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/box_shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/collision/narrow_phase/contact_result.cpp)

target_link_libraries(PETestBoxBoxCollision PEUtils)
