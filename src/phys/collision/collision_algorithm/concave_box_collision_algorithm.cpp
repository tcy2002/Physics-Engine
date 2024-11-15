#include "concave_box_collision_algorithm.h"
#include "concave_convex_collision_algorithm.h"
#include "convex_convex_collision_algorithm.h"
#include "phys/shape/box_shape.h"
#include "phys/shape/concave_mesh_shape.h"

namespace pe_phys_collision {

    bool ConcaveBoxCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                        pe::Transform trans_a, pe::Transform trans_b,
                                                        pe::Real refScale, ContactResult& result) {
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
        auto& edges_box = ((pe_phys_shape::BoxShape*)shape_box)->getUniqueEdges();

        pe::Vector3 sep;
        pe::Real margin = PE_MARGIN;

        VertexArray world_verts_b1;
        VertexArray world_verts_b2;

        pe::Transform trans_box_rel2concave = trans_concave.inverse() * trans_box;
        pe::Vector3 convex_AA, convex_BB;
        shape_box->getAABB(trans_box_rel2concave, convex_AA, convex_BB);
        pe::Array<int> intersect;
        ((pe_phys_shape::ConcaveMeshShape*)shape_concave)->getIntersectFaces(convex_AA, convex_BB, intersect);

        result.setSwapFlag(shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh);
        for (auto fi : intersect) {
            auto& f = mesh_concave.faces[fi];
            pe::Mesh mesh_face;
            pe::Mesh::Face face_face;
            for (int i = 0; i < (int)f.indices.size(); i++) {
                mesh_face.vertices.push_back(mesh_concave.vertices[f.indices[i]]);
                face_face.indices.push_back(i);
            }
            face_face.normal = f.normal;
            mesh_face.faces.push_back(face_face);
            pe_phys_shape::ConvexMeshShape shape_face;
            shape_face._mesh = mesh_face;

            if (!ConvexConvexCollisionAlgorithm::findSeparatingAxis(
                shape_box, &shape_face, mesh_box, mesh_face,
                edges_box, shape_face.getUniqueEdges(),
                trans_box, trans_concave, sep, margin, result)) {
                continue;
            }
            ConvexConvexCollisionAlgorithm::clipHullAgainstHull(
                sep, mesh_box, mesh_face, trans_box, trans_concave,
                -refScale, margin, world_verts_b1, world_verts_b2,
                margin, result);
        }
        result.setSwapFlag(false);

        return true;
    }
    
} // pe_phys_collision