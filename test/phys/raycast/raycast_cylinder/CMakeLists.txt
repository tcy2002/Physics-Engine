add_executable(PETestRaycastCylinder test.cpp
        ${ENGINE_INCLUDE_DIR}/phys/raycast/raycast/raycast_cylinder.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/shape.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/cylinder_shape.cpp
        ${ENGINE_INCLUDE_DIR}/phys/object/rigidbody.cpp)

target_link_libraries(PETestRaycastCylinder PEUtils)