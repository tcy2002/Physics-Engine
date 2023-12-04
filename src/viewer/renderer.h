#pragma once

#include "def.h"

namespace simple_viewer {

// Note: This module is not thread-safe, so the upper module
// should ensure that only one thread is modifying the mesh

// must be consistent with the ObjType in opengl_viewer.h
enum RenderType {
    R_MESH = 1,
    R_LINE = 2
};

/**
 * @brief Basic render
 */
class Renderer {
protected:
    unsigned int VAO, VBO, EBO;
    unsigned long long _vertex_count;
    float *_vertices;
    unsigned long long _triangle_count;
    unsigned int *_indices;

    COMMON_BOOL_GET(inited, Inited)
    COMMON_BOOL_GET(dynamic, Dynamic)
    COMMON_MEMBER_SET_GET(Transform, transform, Transform)
    COMMON_MEMBER_SET_GET(Vector3, color, Color)

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
    void loadMesh(const Mesh& mesh);

public:
    explicit MeshRenderer(const Mesh& mesh, bool dynamic = false);

    int type() const override { return RenderType::R_MESH; }
    void init(int VAP_position, int VAP_normal) override;
    void render() const override;

    bool updateMesh(const Mesh& mesh);
};

/**
 * @brief Line renderer
 */
class LineRenderer : public Renderer {
private:
    COMMON_MEMBER_SET_GET(float, width, Width)

    void loadLine(const std::vector<Vector3>& points);

public:
    explicit LineRenderer(const std::vector<Vector3>& points, bool dynamic = false);

    int type() const override { return RenderType::R_LINE; }
    void init(int VAP_position, int VAP_normal) override;
    void render() const override;

    bool updateLine(const std::vector<Vector3>& points);
};

} // namespace simple_viewer