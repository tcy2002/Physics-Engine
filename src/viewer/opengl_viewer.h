#pragma once

#include "common/vector3.h"
#include "common/mesh.h"
#include "common/transform.h"

namespace pe_viewer {

/**
 * @brief A singleton, single-window, reopenable OpenGL viewer
 *
 *
 */
class OpenglViewer {
public:
    static void open(const std::string& name, int width = 800, int height = 600);
    static void close();

    static void setCamera(const pe_common::Vector3& position, PEReal yaw, PEReal pitch);

    static int addMesh(const pe_common::Mesh& mesh, bool dynamic = false);
    static bool updateMesh(int id, const pe_common::Mesh& mesh);
    static bool updateMeshTransform(int id, const pe_common::Transform& transform);
    static bool updateMeshColor(int id, const pe_common::Vector3& color);
    static void delMesh(int id);
    static void clearMeshes();

private:
    OpenglViewer() = default;
    OpenglViewer(const OpenglViewer&) = delete;
    OpenglViewer& operator=(const OpenglViewer&) = delete;
    ~OpenglViewer();
};

} // namespace pe_viewer