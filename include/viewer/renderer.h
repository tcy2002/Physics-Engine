#pragma once

#include "common/mesh.h"
#include "common/transform.h"
#include "def.h"

namespace pe_viewer {

/**
 * @brief Mesh renderer
 *
 * Note: This class is not thread-safe, so the upper class
 * should ensure that only one thread is modifying the mesh
 */
class MeshRenderer {
private:
    uint32_t VAO, VBO, EBO;
    size_t _vertex_count;
    float *_vertices;
    size_t _triangle_count;
    uint32_t *_indices;
    bool _dynamic;

    PE_BOOL_GET(inited, Inited)
    PE_MEMBER_SET_GET(pe_common::Transform, transform, Transform)
    PE_MEMBER_SET_GET(pe_common::Vector3, color, Color)

    void loadMesh(const pe_common::Mesh& mesh);

public:
    explicit MeshRenderer(const pe_common::Mesh& mesh, bool dynamic = false);
    MeshRenderer(const MeshRenderer& other) = delete;
    ~MeshRenderer();

    void init(int VAP_position, int VAP_normal);
    void deinit();
    void render() const;
    bool updateMesh(const pe_common::Mesh& mesh);
};

} // namespace pe_viewer