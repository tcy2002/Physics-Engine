add_executable(PETestBoxCapsuleCollision test.cpp
        ${ENGINE_INCLUDE_DIR}/physics/collision/collision_algorithm/box_capsule_collision_algorithm.cpp
        ${ENGINE_INCLUDE_DIR}/physics/object/rigidbody.cpp
        ${ENGINE_INCLUDE_DIR}/physics/shape/shape.cpp
        ${ENGINE_INCLUDE_DIR}/physics/shape/box_shape.cpp
        ${ENGINE_INCLUDE_DIR}/physics/shape/capsule_shape.cpp
        ${ENGINE_INCLUDE_DIR}/physics/collision/narrow_phase/contact_result.cpp
        ${ENGINE_INCLUDE_DIR}/physics/fracture/fracture_utils/fracture_data.cpp
        ${ENGINE_INCLUDE_DIR}/physics/fracture/fracture_utils/fracture_utils.cpp
        ${ENGINE_INCLUDE_DIR}/physics/fracture/fracture_utils/fracture_data_manager.cpp)

target_link_libraries(PETestBoxCapsuleCollision PEUtils)
