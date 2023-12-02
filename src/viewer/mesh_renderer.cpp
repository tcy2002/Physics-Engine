#include "mesh_renderer.h"

#include <GL/glew.h>

using namespace pe_viewer;

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
    _inited(false), _dynamic(dynamic),
    _transform(pe_common::Transform::identity()),
    _color(pe_common::Vector3(0.3, 0.25, 0.75)),
    VAO(0), VBO(0), EBO(0) {
    loadMesh(mesh);
}

MeshRenderer::~MeshRenderer() {
    deinit();
    delete[] _vertices;
    delete[] _indices;
}

void MeshRenderer::init() {
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
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 6, 0);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 6, (void *)(sizeof(float) * 3));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizei)(sizeof(uint32_t) * 3 * _triangle_count), _indices, draw_mode);
    _inited = true;
}

void MeshRenderer::deinit() {
    if (!_inited) return;
    _inited = false;
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    glDeleteVertexArrays(1, &VAO);
}

void MeshRenderer::render() const {
    if (!_inited) return;
    glBindVertexArray(VAO);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);
    glDrawElements(GL_TRIANGLES, (GLsizei)(_triangle_count * 3), GL_UNSIGNED_INT, 0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
    glBindVertexArray(0);
}

bool MeshRenderer::updateMesh(const pe_common::Mesh &mesh) {
    if (!_dynamic) return false;
    loadMesh(mesh);
    _inited = false;
    return true;
}
