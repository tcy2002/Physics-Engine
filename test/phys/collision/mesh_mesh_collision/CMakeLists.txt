add_definitions(-DCURRENT_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(PETestMeshMeshCollision test.cpp
        ${ENGINE_INCLUDE_DIR}/phys/collision/collision_algorithm/convex_convex_collision_algorithm.cpp
        ${ENGINE_INCLUDE_DIR}/phys/collision/collision_algorithm/box_box_collision_algorithm.cpp
        ${ENGINE_INCLUDE_DIR}/phys/object/rigidbody.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/convex_mesh_shape.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/box_shape.cpp
        ${ENGINE_INCLUDE_DIR}/phys/collision/narrow_phase/contact_result.cpp
        ${ENGINE_INCLUDE_DIR}/phys/fracture/fracture_utils/fracture_data.cpp
        ${ENGINE_INCLUDE_DIR}/phys/fracture/fracture_utils/fracture_utils.cpp)

target_link_libraries(PETestMeshMeshCollision PEUtils)
