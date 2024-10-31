#include "concave_box_collision_algorithm.h"
#include "concave_convex_collision_algorithm.h"
#include <phys/shape/concave_mesh_shape.h>
#include <phys/shape/convex_mesh_shape.h>
#include <phys/shape/box_shape.h>
#include <phys/shape/default_mesh.h>
#include "convex_convex_collision_algorithm.h"

namespace pe_phys_collision {

    bool ConcaveBoxCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                           pe::Transform trans_a, pe::Transform trans_b,
                                                           ContactResult& result) {
        if (!((shape_a->getType() == pe_phys_shape::ShapeType::Box &&
            shape_b->getType() == pe_phys_shape::ShapeType::ConcaveMesh) ||
            (shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh &&
                shape_b->getType() == pe_phys_shape::ShapeType::Box))) {
            return false;
        }

        auto shape_concave = shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh ? shape_a : shape_b;
        auto shape_box = shape_a->getType() == pe_phys_shape::ShapeType::Box ? shape_a : shape_b;
        auto trans_concave = shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh ? trans_a : trans_b;
        auto trans_box = shape_a->getType() == pe_phys_shape::ShapeType::Box ? trans_a : trans_b;
        auto& mesh_concave = ((pe_phys_shape::ConcaveMeshShape*)shape_concave)->getMesh();
        auto& mesh_box = ((pe_phys_shape::BoxShape*)shape_box)->getMesh();

        pe::Vector3 sep;
        pe::Real margin = PE_MARGIN;

        VertexArray world_verts_b1;
        VertexArray world_verts_b2;

        pe::Transform trans_box_rel2concave = trans_concave.inverse() * trans_box;
        pe::Vector3 convex_AA, convex_BB;
        shape_box->getAABB(trans_box_rel2concave, convex_AA, convex_BB);
        pe::Array<int> intersect;
        ((pe_phys_shape::ConcaveMeshShape*)shape_concave)->getIntersetFaces(convex_AA, convex_BB, intersect);

        result.setSwapFlag(shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh);
        for (auto fi : intersect) {
            auto& f = mesh_concave.faces[fi];
            pe::Array<pe::Vector3> concave_unique_edges;
            ConcaveConvexCollisionAlgorithm::getUniqueEdges(mesh_concave, f, concave_unique_edges);
            if (!ConcaveConvexCollisionAlgorithm::findSeparatingAxis(
                f.normal, shape_box, mesh_concave, f, mesh_box,
                concave_unique_edges,
                pe_phys_shape::_box_unique_edges,
                trans_concave, trans_box, sep, margin, result)) {
                continue;
            }

            world_verts_b1.resize(0);
            for (auto e0 : f.indices) {
                world_verts_b1.push_back(trans_concave * mesh_concave.vertices[e0].position);
            }
            ConvexConvexCollisionAlgorithm::clipFaceAgainstHull(sep, mesh_box, trans_box,
                world_verts_b1, world_verts_b2,
                PE_REAL_MIN, margin, margin, result);
        }

        return true;
    }
    
} // pe_phys_collision