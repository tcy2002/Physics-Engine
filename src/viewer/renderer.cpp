#include "renderer.h"

#include <GL/glew.h>

using namespace pe_viewer;

Renderer::Renderer():
    VAO(0), VBO(0), EBO(0),
    _vertex_count(0), _vertices(nullptr),
    _triangle_count(0), _indices(nullptr),
    _inited(false) {}

Renderer::~Renderer() {
    deinit();
    delete[] _vertices;
    delete[] _indices;
}

void Renderer::deinit() {
    if (!_inited) return;
    _inited = false;
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    if (EBO != 0) glDeleteBuffers(1, &EBO);
}

void MeshRenderer::loadMesh(const pe_common::Mesh &mesh) {
    // load vertex data
    _vertex_count = mesh.vertices.size();
    _vertices = new float[_vertex_count * 6];
    size_t i = 0;
    for (uint32_t j = 0; j < _vertex_count; j++) {
        auto& v = mesh.vertices[j].position;
        auto& n = mesh.vertices[j].normal;
        _vertices[i++] = (float)v.x;
        _vertices[i++] = (float)v.y;
        _vertices[i++] = (float)v.z;
        _vertices[i++] = (float)n.x;
        _vertices[i++] = (float)n.y;
        _vertices[i++] = (float)n.z;
    }

    // load face data, and transform into triangles
    _triangle_count = 0;
    for (auto& f : mesh.faces) {
        _triangle_count += f.indices.size() - 2;
    }
    _indices = new uint32_t[_triangle_count * 3];
    size_t n; i = 0;
    for (auto& f : mesh.faces) {
        n = f.indices.size();
        for (uint32_t j = 1; j < n - 1; j++) {
            _indices[i++] = f.indices[0];
            _indices[i++] = f.indices[j];
            _indices[i++] = f.indices[j + 1];
        }
    }
}

MeshRenderer::MeshRenderer(const pe_common::Mesh& mesh, bool dynamic):
    Renderer(),
    _dynamic(dynamic),
    _transform(pe_common::Transform::identity()),
    _color({0.3, 0.25, 0.8}) {
    loadMesh(mesh);
}

void MeshRenderer::init(int VAP_position, int VAP_normal) {
    if (_inited) return;
    auto draw_mode = _dynamic ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW;
    if (VAO == 0) {
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);
    }
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, (GLsizei)(sizeof(float) * 6 * _vertex_count), _vertices, draw_mode);
    glVertexAttribPointer(VAP_position, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 6, (void*)0);
    glVertexAttribPointer(VAP_normal, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 6, (void *)(sizeof(float) * 3));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizei)(sizeof(uint32_t) * 3 * _triangle_count), _indices, draw_mode);
    _inited = true;
}

void MeshRenderer::render() const {
    if (!_inited) return;
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, (GLsizei)(_triangle_count * 3), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

bool MeshRenderer::updateMesh(const pe_common::Mesh &mesh) {
    if (!_dynamic) return false;
    loadMesh(mesh);
    _inited = false;
    return true;
}

void LineRenderer::loadLine(const pe::Array<pe_common::Vector3>& points) {
    _vertex_count = (points.size() - 1) * 2;
    _vertices = new float[_vertex_count * 3];
    size_t i = 0;
    _vertices[i++] = (float)points.front().x;
    _vertices[i++] = (float)points.front().y;
    _vertices[i++] = (float)points.front().z;
    for (auto p = points.begin() + 1; p != --points.end(); p++) {
        _vertices[i++] = (float)p->x;
        _vertices[i++] = (float)p->y;
        _vertices[i++] = (float)p->z;
        _vertices[i++] = (float)p->x;
        _vertices[i++] = (float)p->y;
        _vertices[i++] = (float)p->z;
    }
    _vertices[i++] = (float)points.back().x;
    _vertices[i++] = (float)points.back().y;
    _vertices[i] = (float)points.back().z;
}

LineRenderer::LineRenderer(const pe::Array<pe_common::Vector3>& points):
    Renderer(),
    _width(1),
    _color({1, 0.95, 0}) {
    loadLine(points);
}

void LineRenderer::init(int VAP_position, int VAP_normal) {
    if (_inited) return;
    if (VAO == 0) {
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
    }
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, (GLsizei)(sizeof(float) * 3 * _vertex_count), _vertices, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(VAP_position, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, (void*)0);
    _inited = true;
}

void LineRenderer::render() const {
    if (!_inited) return;
    glBindVertexArray(VAO);
    glLineWidth((float)_width);
    glDrawArrays(GL_LINES, 0, (GLsizei)(_vertex_count * 3));
    glBindVertexArray(0);
}

void LineRenderer::updateLine(const pe::Array<pe_common::Vector3>& points) {
    loadLine(points);
    _inited = false;
}
