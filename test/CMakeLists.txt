include_directories(${ENGINE_INCLUDE_DIR} .)

set(TEST_EXT_LIBS PECommon)

add_subdirectory(phys)
add_subdirectory(core)
add_subdirectory(common)
