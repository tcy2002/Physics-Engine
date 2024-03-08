#include "convex_mesh_shape.h"
#include <algorithm>
#include "phys/fracture/fracture_utils/fracture_data.h"
#include "utils/hash_vector.h"

namespace pe_phys_shape {
    // the mesh must be convex, and contains all needed properties of a mesh (vertices, faces, normals, etc.)
    void ConvexMeshShape::setMesh(pe::Mesh mesh) {
        _mesh = std::move(mesh);

        // calculate unique edges: each edge is represented by two vertices,
        // and each edge does not appear more than once in the list
        _unique_edges.clear();
        utils::hash_vector<pe::Vector3> vert_map(_mesh.vertices.size(),
                                                 pe_phys_fracture::vector3_hash_func,
                                                 pe_phys_fracture::vector3_equal);
        pe::Map<pe::KV<uint32_t , uint32_t>, bool> edge_map;
        for (auto& f : _mesh.faces) {
            for (int i = 0; i < f.indices.size(); i++) {
                auto v0 = f.indices[i];
                auto v1 = f.indices[(i + 1) % f.indices.size()];
                uint32_t id0 = vert_map.index_of(_mesh.vertices[v0].position);
                uint32_t id1 = vert_map.index_of(_mesh.vertices[v1].position);
                if (id0 == -1) { vert_map.push_back(_mesh.vertices[v0].position); id0 = vert_map.size() - 1; }
                if (id1 == -1) { vert_map.push_back(_mesh.vertices[v1].position); id1 = vert_map.size() - 1; }
                if (id0 > id1) std::swap(id0, id1);
                if (edge_map.find({id0, id1}) == edge_map.end()) {
                    edge_map[{id0, id1}] = true;
                    _unique_edges.push_back(_mesh.vertices[v0].position - _mesh.vertices[v1].position);
                }
            }
        }

        // calculate local center
        _local_center = pe::Vector3::zeros();
        for (auto& v : _mesh.vertices) {
            _local_center += v.position;
        }
        _local_center /= (pe::Real)_mesh.vertices.size();
    }

    pe::Vector3 ConvexMeshShape::localGetSupportVertex(const pe::Vector3 &dir) const {
        pe::Real max_dot = PE_REAL_MIN;
        pe::Vector3 result;
        for (auto &p: _mesh.vertices) {
            auto dot = p.position.dot(dir);
            if (dot > max_dot) {
                max_dot = dot;
                result = p.position;
            }
        }
        return result;
    }

    void ConvexMeshShape::getAABB(const pe::Transform &transform, pe::Vector3 &min, pe::Vector3 &max) const {
        min = PE_VEC_MAX;
        max = PE_VEC_MIN;
        auto &rot = transform.getBasis();
        auto &pos = transform.getOrigin();
        for (auto &p: _mesh.vertices) {
            auto v = rot * p.position;
            min = PE_MIN_VEC(min, v);
            max = PE_MAX_VEC(max, v);
        }
        min += pos;
        max += pos;
    }

    bool ConvexMeshShape::localIsInside(const pe::Vector3 &point) const {
        return !std::any_of(_mesh.faces.begin(), _mesh.faces.end(), [&](auto &f) {
            auto &normal = f.normal;
            auto &p0 = _mesh.vertices[f.indices[0]].position;
            return normal.dot(point - p0) > 0;
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
            minProj = std::min(minProj, v);
            maxProj = std::max(maxProj, v);
        }

        minProj += offset;
        maxProj += offset;
        minPoint = axis * minProj;
        maxPoint = axis * maxProj;
    }

    pe::Matrix3 ConvexMeshShape::calcLocalInertia(pe::Real mass) const {
        // approximate the mesh as a box
        pe::Vector3 min = PE_VEC_MAX, max = PE_VEC_MIN;
        getAABB(pe::Transform(pe::Matrix3::identity(), pe::Vector3::zeros()), min, max);
        pe::Vector3 size = max - min;
        pe::Vector3 center = (min + max) * 0.5;
        // TODO: calculate the inertia of the translated aabb box
        return pe::Matrix3::identity();
    }

}