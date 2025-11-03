add_executable(PETestRigidBody test.cpp
        ${ENGINE_INCLUDE_DIR}/physics/object/rigidbody.cpp
        ${ENGINE_INCLUDE_DIR}/physics/shape/shape.cpp
        ${ENGINE_INCLUDE_DIR}/physics/shape/box_shape.cpp)

target_link_libraries(PETestRigidBody PEUtils)
