#include "convex_mesh_shape.h"
#include <algorithm>

//#define INERTIA_USE_AABB

namespace pe_phys_shape {
    // The mesh must be convex, and has complete properties (vertices, faces, normals).
    // The mesh will be relocated to the centroid of the mesh.
    // Returns the relocation vector.
    void ConvexMeshShape::setMesh(pe::Mesh mesh) {
        _mesh = std::move(mesh);

        // build the bvh search tree
        int node_size = PE_MAX((int)_mesh.faces.size() / 2047, 1);
        _bvh.setMesh(_mesh, node_size);

        // calculate unique edges: each edge is represented by two vertices,
        // and each edge does not appear more than once in the list
        _unique_edges.clear();
        pe::Vector3HashList vert_map((uint32_t)_mesh.vertices.size());
        pe::Map<pe::KV<uint32_t , uint32_t>, bool> edge_map;
        for (auto& f : _mesh.faces) {
            _unique_faces.push_back({});
            for (int i = 0; i < (int)f.indices.size(); i++) {
                auto v0 = f.indices[i];
                auto v1 = f.indices[(i + 1) % f.indices.size()];
                uint32_t id0 = (uint32_t)(vert_map.find(_mesh.vertices[v0].position) - vert_map.begin());
                if (id0 == vert_map.size()) { vert_map.push_back(_mesh.vertices[v0].position); }
                uint32_t id1 = (uint32_t)(vert_map.find(_mesh.vertices[v1].position) - vert_map.begin());
                if (id1 == vert_map.size()) { vert_map.push_back(_mesh.vertices[v1].position); }
                if (i == 0) _unique_faces.back().push_back(id0);
                if (i != (int)f.indices.size() - 1) _unique_faces.back().push_back(id1);
                if (id0 > id1) std::swap(id0, id1);
                if (edge_map.find({id0, id1}) == edge_map.end()) {
                    edge_map[{id0, id1}] = true;
                    _unique_edges.push_back((_mesh.vertices[v0].position - _mesh.vertices[v1].position).normalized());
                }
            }
        }
        _unique_verts = std::move(vert_map.to_vector());
    }

    void ConvexMeshShape::getIntersetFaces(const pe::Vector3& AA, const pe::Vector3& BB, pe::Array<int>& intersect) const {
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
        pe::Matrix3 rot = transform.getBasis();
        pe::Vector3 trans = transform.getOrigin();
        pe::Vector3 local_axis = rot.transposed() * axis;
        pe::Real offset = trans.dot(axis);

        minProj = PE_REAL_MAX;
        maxProj = PE_REAL_MIN;
        for (auto &p: _mesh.vertices) {
            auto v = p.position.dot(local_axis);
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

    pe::Matrix3 ConvexMeshShape::calcLocalInertia(pe::Real mass) const {
#   ifdef INERTIA_USE_AABB
        pe::Vector3 aabb_min, aabb_max;
        getAABB(pe::Transform::identity(), aabb_min, aabb_max);
        pe::Vector3 size = aabb_max - aabb_min;
        pe::Real x2 = size.x * size.x, y2 = size.y * size.y, z2 = size.z * size.z;
        pe::Real mass_12 = mass / 12.0;
        return {
                mass_12 * (y2 + z2), 0, 0,
                0, mass_12 * (x2 + z2), 0,
                0, 0, mass_12 * (x2 + y2)
        };
#   else
#   define INERTIA_INTERNAL_ITER 4
#   define INERTIA_SURFACE_ITER 4

        pe::Array<pe::KV<pe::Vector3, pe::Real>> elements;
        elements.reserve(_unique_edges.size() * INERTIA_SURFACE_ITER * 2 *
                         _unique_verts.size() * INERTIA_SURFACE_ITER);

        pe::Real sum = 0;
        for (auto& face : _unique_faces) {
            pe::Vector3 center_face = pe::Vector3::zeros();
            for (auto& v : face) {
                center_face += _unique_verts[v];
            }
            center_face /= (pe::Real)face.size();

            for (int i = 1; i <= INERTIA_INTERNAL_ITER; i++) {
                pe::Real m = pe::Real(i * i);
                for (int j = 1; j < INERTIA_SURFACE_ITER; j++) {
                    for (auto& v : face) {
                        sum += m * j;
                        elements.push_back({center_face.lerp(_unique_verts[v],
                                                             (pe::Real)j / INERTIA_SURFACE_ITER), m * j});
                    }
                }
            }
        }
        for (int i = 1; i <= INERTIA_INTERNAL_ITER; i++) {
            pe::Real m = pe::Real(i * i * INERTIA_SURFACE_ITER);
            for (auto& v : _unique_verts) {
                sum += m;
                elements.push_back({v * ((pe::Real)i / INERTIA_INTERNAL_ITER), m});
            }
        }

        pe::Matrix3 inertia = pe::Matrix3::zeros();
        for (auto& e : elements) {
            pe::Real x2 = e.first.x * e.first.x;
            pe::Real y2 = e.first.y * e.first.y;
            pe::Real z2 = e.first.z * e.first.z;
            pe::Real xy = e.first.x * e.first.y;
            pe::Real xz = e.first.x * e.first.z;
            pe::Real yz = e.first.y * e.first.z;
            pe::Real m = e.second;
            inertia[0][0] += m * (y2 + z2);
            inertia[1][1] += m * (x2 + z2);
            inertia[2][2] += m * (x2 + y2);
            inertia[0][1] -= m * xy;
            inertia[0][2] -= m * xz;
            inertia[1][2] -= m * yz;
        }
        inertia[1][0] = inertia[0][1];
        inertia[2][0] = inertia[0][2];
        inertia[2][1] = inertia[1][2];
        return inertia * (mass / (2 * sum));
#   endif
    }

    pe::Real ConvexMeshShape::calcMeshVolume(const pe::Mesh &mesh) {
        pe::Real volume = 0;
        for (auto& face : mesh.faces) {
            for (int i = 0; i < (int)face.indices.size() - 2; i++) {
                const pe::Vector3 &v0 = mesh.vertices[face.indices[0]].position;
                const pe::Vector3 &v1 = mesh.vertices[face.indices[i + 1]].position;
                const pe::Vector3 &v2 = mesh.vertices[face.indices[i + 2]].position;
                volume += v0.dot(v1.cross(v2));
            }
        }
        return volume / pe::Real(6.0);
    }

    pe::Vector3 ConvexMeshShape::calcMeshCentroid(const pe::Mesh &mesh) {
        // not accurate
        pe::Vector3 centroid = pe::Vector3::zeros();
        pe::Real surface_area = 0;
        for (auto &f: mesh.faces) {
            pe::Vector3 face_centroid = pe::Vector3::zeros();
            pe::Vector3 face_area = pe::Vector3::zeros();
            for (int i = 0; i < (int)f.indices.size(); i++) {
                auto &p0 = mesh.vertices[f.indices[i]].position;
                auto &p1 = mesh.vertices[f.indices[(i + 1) % f.indices.size()]].position;
                face_area += p0.cross(p1);
                face_centroid += p0;
            }
            pe::Real face_area_ = face_area.norm();
            surface_area += face_area_;
            centroid += face_centroid / (pe::Real) f.indices.size() * face_area_;
        }
        return centroid / surface_area;
    }
}