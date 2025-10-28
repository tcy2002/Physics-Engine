add_executable(PETestRaycastSphere test.cpp
        ${ENGINE_INCLUDE_DIR}/phys/raycast/raycast/raycast_sphere.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/shape.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/sphere_shape.cpp
        ${ENGINE_INCLUDE_DIR}/phys/object/rigidbody.cpp)

target_link_libraries(PETestRaycastSphere PEUtils)