add_definitions(-DCURRENT_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(PETestMeshMeshCollision test.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/collision/collision_algorithm/convex_convex_collision_algorithm.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/collision/collision_algorithm/box_box_collision_algorithm.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/object/rigidbody.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/convex_mesh_shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/box_shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/collision/narrow_phase/contact_result.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/fracture/fracture_utils/fracture_data.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/fracture/fracture_utils/fracture_utils.cpp)

target_link_libraries(PETestMeshMeshCollision PEUtils)
