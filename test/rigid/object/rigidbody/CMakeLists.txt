add_executable(PETestRigidBody test.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/object/rigidbody.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/box_shape.cpp)

target_link_libraries(PETestRigidBody PEUtils)
