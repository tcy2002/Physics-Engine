add_executable(PETestRaycastCylinder test.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/raycast/raycast/raycast_cylinder.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/cylinder_shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/object/rigidbody.cpp)

target_link_libraries(PETestRaycastCylinder PEUtils)