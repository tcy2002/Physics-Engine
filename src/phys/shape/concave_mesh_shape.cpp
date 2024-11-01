#include "concave_mesh_shape.h"

namespace pe_phys_shape {

    void ConcaveMeshShape::setMesh(pe::Mesh mesh) {
        _mesh = std::move(mesh);

        // build the bvh search tree
        int node_size = PE_MAX((int)_mesh.faces.size() / 1023, 1);
        _bvh.setMesh(_mesh, node_size);
    }

} // namespace pe_phys_shape