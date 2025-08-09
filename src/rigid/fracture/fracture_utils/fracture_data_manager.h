#pragma once

#include "fracture_data.h"

namespace pe_phys_fracture {

    /**
     * @brief The mesh manager class, which is used to manage the
     * mesh data, including vertices, triangles, faces(2d polygons)
     * and tetrahedrons.
     *
     * Data of vertices, triangles and faces are stored in hash_vector,
     * since they need both sequence information and random access
     * efficiency.
     *
     * Only support convex mesh, no limitation on other aspects.
     *
     * A general mesh has some features:
     * - it's composed of polygonal faces, and each face consists of
     *   several triangles;
     * - each vertices has its own normal, but all the triangles of a
     *   face share the same normal;
     * - each vertices of a face must be in the same plane, and the
     *   positions must differ, but vertices of different faces may be
     *   the same position, as long as they have different normals.
     * This structure can automatically cater to these features, and
     * provide some efficiency for the upper layer.
     */
    class FractureDataManager {
    private:
        VertexHashList _vertices;
        TriangleHashList _triangles;
        PolygonHashList _faces;
        pe::Array<tetrahedron> _tetrahedrons;

    public:
        FractureDataManager() {}

        void clear() { _vertices.clear(); _triangles.clear(); _faces.clear(); _tetrahedrons.clear(); }

        uint32_t vertex_count() const { return _vertices.size(); }
        uint32_t add_vertex(const pe::Vector3& p, const pe::Vector3& n = pe::Vector3::Zero());
        vertex get_vertex(uint32_t idx) { return _vertices[idx]; }
        void remove_vertex(uint32_t idx, bool direct = false);
        void clear_vertices() { _vertices.clear(); }

        uint32_t triangle_count() const { return (uint32_t)_triangles.size(); }
        uint32_t add_triangle(uint32_t v1, uint32_t v2, uint32_t v3);
        triangle get_triangle(uint32_t idx) { return _triangles[idx]; }
        void remove_triangle(uint32_t idx, bool direct = false);
        void clear_triangles() { _triangles.clear(); }

        uint32_t face_count() const { return (uint32_t)_faces.size(); }
        uint32_t add_face(const polygon& face) { _faces.push_back(face); return _faces.size() - 1; }
        polygon get_face(uint32_t idx) { return _faces[idx]; }
        void remove_face(uint32_t idx) { _faces.erase(_faces.begin() + idx); }
        void clear_faces() { _faces.clear(); }

        uint32_t tetrahedron_count() const { return (uint32_t)_tetrahedrons.size(); }
        uint32_t add_tetrahedron(uint32_t t1, uint32_t t2, uint32_t t3, uint32_t t4,
                                 uint32_t v1, uint32_t v2, uint32_t v3, uint32_t v4);
        tetrahedron get_tetrahedron(uint32_t idx) { return _tetrahedrons[idx]; }
        void remove_tetrahedron(uint32_t idx) { _tetrahedrons.erase(_tetrahedrons.begin() + idx); }
        void clear_tetrahedrons() { _tetrahedrons.clear(); }

        void faces_to_triangles();
        bool add_triangle_to_face(uint32_t v1, uint32_t v2, uint32_t v3);
        void import_from_mesh(const pe::Mesh& mesh);
        void export_to_mesh(pe::Mesh& mesh);
    };

} // namespace pe_phys_fracture