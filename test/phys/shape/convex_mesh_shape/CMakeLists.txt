add_definitions(-DCURRENT_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(PETestCvmShape test.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/convex_mesh_shape.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/box_shape.cpp
        ${ENGINE_INCLUDE_DIR}/phys/shape/cylinder_shape.cpp
        ${ENGINE_INCLUDE_DIR}/phys/fracture/fracture_utils/fracture_data.cpp
        ${ENGINE_INCLUDE_DIR}/phys/fracture/fracture_utils/fracture_utils.cpp)

target_link_libraries(PETestCvmShape PEUtils)
