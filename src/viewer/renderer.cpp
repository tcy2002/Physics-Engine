#include "renderer.h"

#include <GL/glew.h>
#include "default_mesh.h"

using namespace simple_viewer;

Renderer::Renderer():
    VAO(0), VBO(0), EBO(0),
    _vertex_count(0), _vertices(nullptr),
    _triangle_count(0), _indices(nullptr),
    _inited(false), _dynamic(false),
    _transform(Transform::identity()),
    _color({1, 1, 1}) {}

Renderer::~Renderer() {
    deinit();
    delete[] _vertices;
    delete[] _indices;
}

void Renderer::init(int VAP_position, int VAP_normal) {
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
    glEnableVertexAttribArray(VAP_position);
    glEnableVertexAttribArray(VAP_normal);
    glVertexAttribPointer(VAP_position, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 6, (void*)0);
    glVertexAttribPointer(VAP_normal, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 6, (void *)(sizeof(float) * 3));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizei)(sizeof(unsigned int) * 3 * _triangle_count), _indices, draw_mode);
    glBindVertexArray(0);
    _inited = true;
}

void Renderer::deinit() {
    if (VAO == 0) return;
    glDeleteVertexArrays(1, &VAO); VAO = 0;
    glDeleteBuffers(1, &VBO); VBO = 0;
    glDeleteBuffers(1, &EBO); EBO = 0;
    _inited = false;
}

void MeshRenderer::loadMesh(const Mesh& mesh) {
    // load vertex data
    _vertex_count = mesh.vertices.size();
    _vertices = new float[_vertex_count * 6];
    unsigned long long i = 0;
    for (unsigned int j = 0; j < _vertex_count; j++) {
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
    _indices = new unsigned int[_triangle_count * 3];
    unsigned long long n; i = 0;
    for (auto& f : mesh.faces) {
        n = f.indices.size();
        for (unsigned int j = 1; j < n - 1; j++) {
            _indices[i++] = f.indices[0];
            _indices[i++] = f.indices[j];
            _indices[i++] = f.indices[j + 1];
        }
    }
}

MeshRenderer::MeshRenderer(const Mesh& mesh, bool dynamic):
    Renderer() {
    loadMesh(mesh);
    _dynamic = dynamic;
    _color = {0.3, 0.25, 0.8};
}

void MeshRenderer::render() {
    if (!_inited) return;
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, (GLsizei)(_triangle_count * 3), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

bool MeshRenderer::updateMesh(const Mesh& mesh) {
    if (!_dynamic) return false;
    loadMesh(mesh);
    _inited = false;
    return true;
}

void CubeRenderer::loadCube(const Vector3 &size) {
    // load vertex data
    _vertex_count = _cube_mesh.vertices.size();
    _vertices = new float[_vertex_count * 6];
    unsigned long long i = 0;
    for (unsigned int j = 0; j < _vertex_count; j++) {
        auto& v = _cube_mesh.vertices[j].position;
        auto& n = _cube_mesh.vertices[j].normal;
        _vertices[i++] = (float)(v.x * size.x);
        _vertices[i++] = (float)(v.y * size.y);
        _vertices[i++] = (float)(v.z * size.z);
        _vertices[i++] = (float)(n.x * size.x);
        _vertices[i++] = (float)(n.y * size.y);
        _vertices[i++] = (float)(n.z * size.z);
    }

    // load face data, and transform into triangles
    _triangle_count = 0;
    for (auto& f : _cube_mesh.faces) {
        _triangle_count += f.indices.size() - 2;
    }
    _indices = new unsigned int[_triangle_count * 3];
    unsigned long long n; i = 0;
    for (auto& f : _cube_mesh.faces) {
        n = f.indices.size();
        for (unsigned int j = 1; j < n - 1; j++) {
            _indices[i++] = f.indices[0];
            _indices[i++] = f.indices[j];
            _indices[i++] = f.indices[j + 1];
        }
    }
}

CubeRenderer::CubeRenderer(const Vector3 &size, bool dynamic) {
    loadCube(size);
    _dynamic = dynamic;
    _color = {0.3, 0.25, 0.8};
}

void CubeRenderer::render() {
    if (!_inited) return;
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, (GLsizei)(_triangle_count * 3), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

bool CubeRenderer::updateCube(const Vector3 &size) {
    if (!_dynamic) return false;
    loadCube(size);
    _inited = false;
    return true;
}

void CylinderRenderer::loadCylinder(float radius, float height) {
    // load vertex data
    radius *= 2;
    _vertex_count = _cylinder_mesh.vertices.size();
    _vertices = new float[_vertex_count * 6];
    unsigned long long i = 0;
    for (unsigned int j = 0; j < _vertex_count; j++) {
        auto& v = _cylinder_mesh.vertices[j].position;
        auto& n = _cylinder_mesh.vertices[j].normal;
        _vertices[i++] = (float)(v.x * radius);
        _vertices[i++] = (float)(v.y * height);
        _vertices[i++] = (float)(v.z * radius);
        _vertices[i++] = (float)(n.x * radius);
        _vertices[i++] = (float)(n.y * height);
        _vertices[i++] = (float)(n.z * radius);
    }

    // load face data, and transform into triangles
    _triangle_count = 0;
    for (auto& f : _cylinder_mesh.faces) {
        _triangle_count += f.indices.size() - 2;
    }
    _indices = new unsigned int[_triangle_count * 3];
    unsigned long long n; i = 0;
    for (auto& f : _cylinder_mesh.faces) {
        n = f.indices.size();
        for (unsigned int j = 1; j < n - 1; j++) {
            _indices[i++] = f.indices[0];
            _indices[i++] = f.indices[j];
            _indices[i++] = f.indices[j + 1];
        }
    }
}

CylinderRenderer::CylinderRenderer(float radius, float height, bool dynamic) {
    loadCylinder(radius, height);
    _dynamic = dynamic;
    _color = {0.3, 0.25, 0.8};
}

void CylinderRenderer::render() {
    if (!_inited) return;
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, (GLsizei)(_triangle_count * 3), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

bool CylinderRenderer::updateCylinder(float radius, float height) {
    if (!_dynamic) return false;
    loadCylinder(radius, height);
    _inited = false;
    return true;
}

void LineRenderer::loadLine(const std::vector<Vector3>& points) {
    _vertex_count = (points.size() - 1) * 2;
    _vertices = new float[_vertex_count * 6]{};
    unsigned long long i = 0;
    _vertices[i++] = (float)points.front().x;
    _vertices[i++] = (float)points.front().y;
    _vertices[i++] = (float)points.front().z;
    i += 3;
    for (auto p = points.begin() + 1; p != --points.end(); p++) {
        _vertices[i++] = (float)p->x;
        _vertices[i++] = (float)p->y;
        _vertices[i++] = (float)p->z;
        i += 3;
        _vertices[i++] = (float)p->x;
        _vertices[i++] = (float)p->y;
        _vertices[i++] = (float)p->z;
        i += 3;
    }
    _vertices[i++] = (float)points.back().x;
    _vertices[i++] = (float)points.back().y;
    _vertices[i] = (float)points.back().z;
}

LineRenderer::LineRenderer(const std::vector<Vector3>& points, bool dynamic):
    Renderer(), _width(1) {
    loadLine(points);
    _dynamic = dynamic;
    _color = {1, 0.95, 0};
}

void LineRenderer::render() {
    if (!_inited) return;
    glBindVertexArray(VAO);
    glLineWidth((float)_width);
    glDrawArrays(GL_LINES, 0, (GLsizei)(_vertex_count * 3));
    glBindVertexArray(0);
}

bool LineRenderer::updateLine(const std::vector<Vector3>& points) {
    if (!_dynamic) return false;
    loadLine(points);
    _inited = false;
    return true;
}
