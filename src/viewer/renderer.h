#pragma once

#include "common/mesh.h"
#include "common/transform.h"
#include "def.h"

namespace pe_viewer {

// Note: This module is not thread-safe, so the upper module
// should ensure that only one thread is modifying the mesh

enum RenderType { R_MESH, R_LINE };

/**
 * @brief Basic render
 */
class Renderer {
protected:
    uint32_t VAO, VBO, EBO;
    size_t _vertex_count;
    float *_vertices;
    size_t _triangle_count;
    uint32_t *_indices;

    PE_BOOL_GET(inited, Inited)

public:
    Renderer();
    Renderer(const Renderer& other) = delete;
    virtual ~Renderer();

    virtual int type() const = 0;
    virtual void init(int VAP_position, int VAP_normal) = 0;
    void deinit();
    virtual void render() const = 0;
};

/**
 * @brief Mesh renderer
 */
class MeshRenderer : public Renderer {
private:
    PE_BOOL_GET(dynamic, Dynamic)
    PE_BOOL_SET_GET(showLines, ShowLines)
    PE_MEMBER_SET_GET(pe_common::Transform, transform, Transform)
    PE_MEMBER_SET_GET(pe_common::Vector3, color, Color)

    void loadMesh(const pe_common::Mesh& mesh);

public:
    explicit MeshRenderer(const pe_common::Mesh& mesh, bool dynamic = false);

    int type() const override { return RenderType::R_MESH; }
    void init(int VAP_position, int VAP_normal) override;
    void render() const override;

    bool updateMesh(const pe_common::Mesh& mesh);
};

/**
 * @brief Line renderer
 */
class LineRenderer : public Renderer {
private:
    PE_MEMBER_SET_GET(PEReal, width, Width)
    PE_MEMBER_SET_GET(pe_common::Vector3, color, Color)

    void loadLine(const pe::Array<pe_common::Vector3>& points);

public:
    explicit LineRenderer(const pe::Array<pe_common::Vector3>& points);

    int type() const override { return RenderType::R_LINE; }
    void init(int VAP_position, int VAP_normal) override;
    void render() const override;

    void updateLine(const pe::Array<pe_common::Vector3>& points);
};

} // namespace pe_viewer