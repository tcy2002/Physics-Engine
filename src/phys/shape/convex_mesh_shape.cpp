#include "convex_mesh_shape.h"
#include <algorithm>

//#define PE_MESH_PROPERTIES_USE_LOCAL_AABB

// style-checked
namespace pe_phys_shape {

    // The mesh must be convex, and has complete properties (vertices, faces, normals).
    // The mesh will be relocated to the center of mass.
    // Returns the relocation vector.
    pe::Vector3 ConvexMeshShape::setMesh(pe::Mesh mesh) {
        _mesh = std::move(mesh);

#   ifdef PE_MESH_PROPERTIES_USE_LOCAL_AABB
        pe::Vector3 aabb_min, aabb_max;
        getAABB(pe::Transform::identity(), aabb_min, aabb_max);
        pe::Vector3 size = aabb_max - aabb_min;
        pe::Real x2 = size.x * size.x, y2 = size.y * size.y, z2 = size.z * size.z;
        _volume = size.x * size.y * size.z;
        _local_inertia = {
                (y2 + z2) / 12, 0, 0,
                0, (x2 + z2) / 12, 0,
                0, 0, (x2 + y2) / 12
        };
        pe::Vector3 cm = (aabb_max + aabb_min) / 2;
#   else

#   define SUB_EXPRESSIONS(w0, w1, w2, f1, f2, f3, g0, g1, g2) do { \
        pe::Real tmp0 = (w0) + (w1); \
        f1 = tmp0 + (w2); \
        pe::Real tmp1 = (w0) * (w0); \
        pe::Real tmp2 = tmp1 + (w1) * tmp0; \
        f2 = tmp2 + (w2) * f1; \
        f3 = (w0) * tmp1 + (w1) * tmp2 + (w2) * f2; \
        g0 = f2 + (w0) * (f1 + (w0)); \
        g1 = f2 + (w1) * (f1 + (w1)); \
        g2 = f2 + (w2) * (f1 + (w2)); \
    } while (0)

        constexpr pe::Real mult[10] = {
            R(1.0) / 6, R(1.0) / 24, R(1.0) / 24, R(1.0) / 24, R(1.0) / 60,
            R(1.0) / 60, R(1.0) / 60, R(1.0) / 120, R(1.0) / 120, R(1.0) / 120
        };
        pe::Real intg[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        for (auto& face : _mesh.faces) {
            auto& v0 = _mesh.vertices[face.indices[0]].position;
            for (int i = 0; i < I(face.indices.size()) - 2; i++) {
                auto& v1 = _mesh.vertices[face.indices[i + 1]].position;
                auto& v2 = _mesh.vertices[face.indices[i + 2]].position;

                // get vertices of triangle t
                pe::Real x0 = v0.x, y0 = v0.y, z0 = v0.z;
                pe::Real x1 = v1.x, y1 = v1.y, z1 = v1.z;
                pe::Real x2 = v2.x, y2 = v2.y, z2 = v2.z;

                // get edges and cross product of edges
                pe::Real a1 = x1 - x0, b1 = y1 - y0, c1 = z1 - z0;
                pe::Real a2 = x2 - x0, b2 = y2 - y0, c2 = z2 - z0;
                pe::Real d0 = b1 * c2 - b2 * c1;
                pe::Real d1 = a2 * c1 - a1 * c2;
                pe::Real d2 = a1 * b2 - a2 * b1;

                // compute integral terms
                pe::Real f1x, f2x, f3x, g0x, g1x, g2x;
                SUB_EXPRESSIONS(x0, x1, x2, f1x, f2x, f3x, g0x, g1x, g2x);
                pe::Real f1y, f2y, f3y, g0y, g1y, g2y;
                SUB_EXPRESSIONS(y0, y1, y2, f1y, f2y, f3y, g0y, g1y, g2y);
                pe::Real f1z, f2z, f3z, g0z, g1z, g2z;
                SUB_EXPRESSIONS(z0, z1, z2, f1z, f2z, f3z, g0z, g1z, g2z);

                // update integrals
                intg[0] += d0 * f1x;
                intg[1] += d0 * f2x;
                intg[2] += d1 * f2y;
                intg[3] += d2 * f2z;
                intg[4] += d0 * f3x;
                intg[5] += d1 * f3y;
                intg[6] += d2 * f3z;
                intg[7] += d0 * (y0 * g0x + y1 * g1x + y2 * g2x);
                intg[8] += d1 * (z0 * g0y + z1 * g1y + z2 * g2y);
                intg[9] += d2 * (x0 * g0z + x1 * g1z + x2 * g2z);
            }
        }
        for (int i = 0; i < 10; i++) {
            intg[i] *= mult[i];
        }

        _volume = intg[0];
        pe::Vector3 cm = {intg[1] / _volume, intg[2] / _volume, intg[3] / _volume};

        // inertia tensor relative to center of mass
        _local_inertia[0][0] = intg[5] + intg[6] - _volume * (cm.y * cm.y + cm.z * cm.z);
        _local_inertia[1][1] = intg[4] + intg[6] - _volume * (cm.z * cm.z + cm.x * cm.x);
        _local_inertia[2][2] = intg[4] + intg[5] - _volume * (cm.x * cm.x + cm.y * cm.y);
        _local_inertia[0][1] = _local_inertia[1][0] = -intg[7] + _volume * cm.x * cm.y;
        _local_inertia[1][2] = _local_inertia[2][1] = -intg[8] + _volume * cm.y * cm.z;
        _local_inertia[0][2] = _local_inertia[2][0] = -intg[9] + _volume * cm.z * cm.x;
        _local_inertia /= _volume;
#   endif
        for (auto& v : _mesh.vertices) {
            v.position -= cm;
        }

        // build the bvh search tree
        int node_size = PE_MAX((int)_mesh.faces.size() / 8191, 1);
        _bvh.setMesh(_mesh, node_size);

        // calculate unique edges: each edge is represented by two vertices,
        _unique_edges.clear();
        pe::Vector3HashList vert_map((uint32_t)_mesh.vertices.size());
        pe::Map<pe::KV<uint32_t , uint32_t>, bool> edge_map;
        for (auto& f : _mesh.faces) {
            for (int i = 0; i < I(f.indices.size()); i++) {
                auto v0 = f.indices[i];
                auto v1 = f.indices[(i + 1) % f.indices.size()];
                int id0 = I(vert_map.find(_mesh.vertices[v0].position) - vert_map.begin());
                if (id0 == vert_map.size()) { vert_map.push_back(_mesh.vertices[v0].position); }
                int id1 = I(vert_map.find(_mesh.vertices[v1].position) - vert_map.begin());
                if (id1 == vert_map.size()) { vert_map.push_back(_mesh.vertices[v1].position); }
                if (id0 > id1) std::swap(id0, id1);
                if (edge_map.find({id0, id1}) == edge_map.end()) {
                    edge_map[{id0, id1}] = true;
                    _unique_edges.emplace_back(_mesh.vertices[v0].position, _mesh.vertices[v1].position);
                }
            }
        }

        // return the relocation vector
        return cm;
    }

    void ConvexMeshShape::getIntersectFaces(const pe::Vector3& AA, const pe::Vector3& BB, pe::Array<int>& intersect) const {
        _bvh.search(AA, BB, intersect);
    }

    void ConvexMeshShape::getAABB(const pe::Transform &transform, pe::Vector3 &min, pe::Vector3 &max) const {
        min = PE_VEC_MAX;
        max = PE_VEC_MIN;
        auto &rot = transform.getBasis();
        auto &pos = transform.getOrigin();
        for (auto &p: _mesh.vertices) {
            auto v = rot * p.position;
            min = pe::Vector3::min2(min, v);
            max = pe::Vector3::max2(max, v);
        }
        min += pos;
        max += pos;
    }

    bool ConvexMeshShape::localIsInside(const pe::Vector3 &point) const {
        return !std::any_of(_mesh.faces.begin(), _mesh.faces.end(), [&](auto &f) {
            auto &normal = f.normal;
            auto &p0 = _mesh.vertices[f.indices[0]].position;
            return normal.dot(point - p0) >= 0;
        });
    }

    void ConvexMeshShape::project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                                  pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const {
        const pe::Matrix3 rot = transform.getBasis();
        const pe::Vector3 trans = transform.getOrigin();
        const pe::Vector3 local_axis = rot.transposed() * axis;
        const pe::Real offset = trans.dot(axis);

        minProj = PE_REAL_MAX;
        maxProj = PE_REAL_MIN;
        for (auto &p: _mesh.vertices) {
            const auto v = p.position.dot(local_axis);
            if (v < minProj) {
                minProj = v;
                minPoint = p.position;
            } else if (v > maxProj) {
                maxProj = v;
                maxPoint = p.position;
            }
        }

        minProj += offset;
        maxProj += offset;
        minPoint = transform * minPoint;
        maxPoint = transform * maxPoint;
    }
}