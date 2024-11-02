#include "concave_mesh_shape.h"

namespace pe_phys_shape {

    pe::Vector3 ConcaveMeshShape::setMesh(pe::Mesh mesh) {
        _mesh = std::move(mesh);

        // inertia and volume is not supported for concave mesh
        _local_inertia = pe::Matrix3::identity();
        _volume = 0;

        // build the bvh search tree
        int node_size = PE_MAX((int)_mesh.faces.size() / 2047, 1);
        _bvh.setMesh(_mesh, node_size);

        return pe::Vector3::zeros();
    }

} // namespace pe_phys_shape