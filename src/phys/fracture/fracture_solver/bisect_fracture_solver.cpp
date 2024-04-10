#include "bisect_fracture_solver.h"
#include "phys/shape/convex_mesh_shape.h"
#include "phys/shape/box_shape.h"

namespace pe_phys_fracture {

    void BisectFractureSolver::solve(const pe::Array<FractureSource>& sources) {
        if (_fracturable_object == 0 || sources.empty()) return;

        // generate points
        pe::Array<pe::Vector3> points;
        pe::Array<pe::Vector3> forces;
        if (!generatePoints(sources, points, forces)) return;

        // retrieve mesh data from different shapes
        pe_phys_shape::Shape* shape = _fracturable_object->getCollisionShape();
        pe::Mesh mesh;
        if (shape->getType() == pe_phys_shape::ShapeType::ConvexMesh) {
            mesh = ((pe_phys_shape::ConvexMeshShape*)(shape))->getMesh();
        } else if (shape->getType() == pe_phys_shape::ShapeType::Box) {
            mesh = ((pe_phys_shape::BoxShape*)(shape))->getMesh();
        } else return;

        pe::Transform world_trans = _fracturable_object->getTransform();

        // generate new rigidbodies
        pe::Array<pe::Mesh> fragments;
        _voronoi->triangulate(points);

    }

} // namespace pe_phys_fracture