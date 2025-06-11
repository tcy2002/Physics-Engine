add_definitions(-DCURRENT_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(PETestCylinderShape test.cpp ${ENGINE_INCLUDE_DIR}/rigid/shape/cylinder_shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/convex_mesh_shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/fracture/fracture_utils/fracture_data.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/fracture/fracture_utils/fracture_utils.cpp)

target_link_libraries(PETestCylinderShape PEUtils)
