add_executable(PETestBoxShape test.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/shape.cpp
        ${ENGINE_INCLUDE_DIR}/rigid/shape/box_shape.cpp)

target_link_libraries(PETestBoxShape PEUtils)
