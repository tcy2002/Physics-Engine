add_executable(PETestPool test.cpp
        ${ENGINE_INCLUDE_DIR}/phys/constraint/constraint/friction_contact_constraint.cpp
        ${ENGINE_INCLUDE_DIR}/phys/object/rigidbody.cpp
        ${ENGINE_INCLUDE_DIR}/phys/collision/narrow_phase/contact_result.cpp)

target_link_libraries(PETestPool ${TEST_EXT_LIBS})