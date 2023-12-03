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

    //// Camera
public:
    static void setCamera(const pe_common::Vector3& position, PEReal yaw, PEReal pitch);

    //// Mesh
public:
    static int addMesh(const pe_common::Mesh& mesh, bool dynamic = false);
    static bool updateMesh(int id, const pe_common::Mesh& mesh);
    static bool updateMeshTransform(int id, const pe_common::Transform& transform);
    static bool updateMeshColor(int id, const pe_common::Vector3& color);
    static void delMesh(int id);
    static void clearMeshes();

    //// Line
public:
    static int addLine(const pe_common::Vector3& start, const pe_common::Vector3& end);
    static bool updateLine(int id, const pe_common::Vector3& start, const pe_common::Vector3& end);
    static bool updateLineWidth(int id, PEReal width);
    static bool updateLineColor(int id, const pe_common::Vector3& color);
    static void delLine(int id);
    static void clearLines();

    //// Point
public:
    static int addPoint(const pe_common::Vector3& position);
    static bool updatePoint(int id, const pe_common::Vector3& position);
    static bool updatePointSize(int id, PEReal size);
    static bool updatePointColor(int id, const pe_common::Vector3& color);
    static void delPoint(int id);
    static void clearPoints();

    //// TODO: Text Panel

private:
    OpenglViewer() = default;
    OpenglViewer(const OpenglViewer&) = delete;
    OpenglViewer& operator=(const OpenglViewer&) = delete;
    ~OpenglViewer();
};

} // namespace pe_viewer