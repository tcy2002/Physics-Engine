include_directories(${ENGINE_INCLUDE_DIR} .)

set(TEST_EXT_LIBS PECommon)

add_subdirectory(common/logger)
add_subdirectory(common/matrix3x3)
add_subdirectory(common/thread_pool)
add_subdirectory(common/vector3)
add_subdirectory(common/transform)
add_subdirectory(core/viewer)
add_subdirectory(phys/object/rigidbody)
add_subdirectory(phys/shape/box_shape)
add_subdirectory(phys/shape/convex_mesh_shape)
