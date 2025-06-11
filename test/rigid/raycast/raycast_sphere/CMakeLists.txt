add_executable(PETestRaycastSphere test.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/raycast/raycast/raycast_sphere.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/sphere_shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/object/rigidbody.cpp)

target_link_libraries(PETestRaycastSphere PEUtils)