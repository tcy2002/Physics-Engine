add_executable(PETestRaycastMesh test.cpp
        ${ENGINE_INCLUDE_DIR}/phys/raycast/raycast/raycast_mesh.cpp
        ${ENGINE_INCLUDE_DIR}/phys/raycast/raycast/raycast_box.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/convex_mesh_shape.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/concave_mesh_shape.cpp
        ${ENGINE_INCLUDE_DIR}/phys/object/rigidbody.cpp)