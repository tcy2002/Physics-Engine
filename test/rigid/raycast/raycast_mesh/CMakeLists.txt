add_executable(PETestRaycastMesh test.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/raycast/raycast/raycast_mesh.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/raycast/raycast/raycast_box.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/convex_mesh_shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/compound_shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/object/rigidbody.cpp)

target_link_libraries(PETestRaycastMesh PEUtils)