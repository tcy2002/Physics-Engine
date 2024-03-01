add_executable(PETestFracture test.cpp
        ${ENGINE_INCLUDE_DIR}/phys/fracture/fracture_data_manager.cpp
        ${ENGINE_INCLUDE_DIR}/phys/fracture/fracture_solver.cpp
        ${ENGINE_INCLUDE_DIR}/phys/fracture/voronoi_calculator.cpp
        ${ENGINE_INCLUDE_DIR}/phys/fracture/fracture_calculator.cpp
        ${ENGINE_INCLUDE_DIR}/phys/object/rigidbody.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/box_shape.cpp)

target_link_libraries(PETestFracture ${TEST_EXT_LIBS})
