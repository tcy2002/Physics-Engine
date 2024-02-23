add_executable(PETestRigidBody test.cpp
        ${ENGINE_INCLUDE_DIR}/phys/object/collision_object.cpp
        ${ENGINE_INCLUDE_DIR}/phys/object/rigidbody.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/box_shape.cpp)

target_link_libraries(PETestRigidBody ${TEST_EXT_LIBS})
