add_executable(PETestRaycastBox test.cpp
        ${ENGINE_INCLUDE_DIR}/phys/raycast/raycast/raycast_box.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/box_shape.cpp
        ${ENGINE_INCLUDE_DIR}/phys/object/rigidbody.cpp)

target_link_libraries(PETestRaycastBox PEUtils)