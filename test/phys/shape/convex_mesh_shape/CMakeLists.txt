add_executable(PETestCvmShape test.cpp ${ENGINE_INCLUDE_DIR}/phys/shape/convex_mesh_shape.cpp)

target_link_libraries(PETestCvmShape ${TEST_EXT_LIBS})
