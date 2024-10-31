#include "concave_mesh_shape.h"

namespace pe_phys_shape {

    pe::Vector3 ConcaveMeshShape::setMesh(pe::Mesh mesh) {
        _mesh = std::move(mesh);

        pe::Vector3 centroid = calcMeshCentroid(_mesh);
        for (auto& v : _mesh.vertices) {
            v.position -= centroid;
        }

        // build the bvh search tree
        int node_size = PE_MAX((int)_mesh.faces.size() / 1023, 1);
        _bvh.setMesh(_mesh, node_size);

        return centroid;
    }

} // namespace pe_phys_shape