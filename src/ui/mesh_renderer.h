#pragma once

#include "def.h"
#include "cg/mesh.h"
#include "cg/transform.h"

namespace pe_ui {

class MeshRenderer {
private:
    uint32_t VAO, VBO, EBO;
    size_t _vertex_count;
    float *_vertices;
    size_t _triangle_count;
    uint32_t *_indices;

    MEMBER_SET_GET(bool, visible, Visible)
    MEMBER_SET_GET(pe_cg::Transform, transform, Transform)
    MEMBER_SET_GET(pe_cg::Vector3, color, Color)

    void loadMesh(const pe_cg::Mesh& mesh);

public:
    explicit MeshRenderer(const pe_cg::Mesh& mesh);
    MeshRenderer(const MeshRenderer& other) = delete;
    ~MeshRenderer();

    void render() const;
};

} // namespace ui