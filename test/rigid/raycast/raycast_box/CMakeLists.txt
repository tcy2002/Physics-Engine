add_executable(PETestRaycastBox test.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/raycast/raycast/raycast_box.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/box_shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/object/rigidbody.cpp)

target_link_libraries(PETestRaycastBox PEUtils)