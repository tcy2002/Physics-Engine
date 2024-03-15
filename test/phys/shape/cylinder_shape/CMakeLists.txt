add_executable(PETestCylinderShape test.cpp ${ENGINE_INCLUDE_DIR}/phys/shape/cylinder_shape.cpp)

target_link_libraries(PETestCylinderShape ${TEST_EXT_LIBS})
