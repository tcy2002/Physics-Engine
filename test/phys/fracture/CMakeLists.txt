add_executable(PETestFracture test.cpp
        ${ENGINE_INCLUDE_DIR}/phys/fracture/fracture_utils/fracture_data.cpp
        ${ENGINE_INCLUDE_DIR}/phys/fracture/fracture_utils/fracture_data_manager.cpp
        ${ENGINE_INCLUDE_DIR}/phys/fracture/fracture_solver/fracture_solver.cpp
        ${ENGINE_INCLUDE_DIR}/phys/fracture/fracture_solver/simple_fracture_solver.cpp
        ${ENGINE_INCLUDE_DIR}/phys/fracture/fracture_utils/fracture_utils.cpp
        ${ENGINE_INCLUDE_DIR}/phys/fracture/voronoi_calculator/voronoi_calculator.cpp
        ${ENGINE_INCLUDE_DIR}/phys/fracture/voronoi_calculator/bowyer_watson_voronoi_calculator.cpp
        ${ENGINE_INCLUDE_DIR}/phys/object/rigidbody.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/convex_mesh_shape.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/box_shape.cpp)

target_link_libraries(PETestFracture PEUtils)
