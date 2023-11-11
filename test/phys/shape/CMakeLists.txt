include_directories(.)

add_definitions(-DSHAPE_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_subdirectory(box_shape)
add_subdirectory(convex_mesh_shape)
