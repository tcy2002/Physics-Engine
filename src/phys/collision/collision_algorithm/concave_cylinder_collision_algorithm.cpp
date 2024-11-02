#include "concave_cylinder_collision_algorithm.h"
#include "concave_convex_collision_algorithm.h"
#include "convex_convex_collision_algorithm.h"
#include "phys/shape/cylinder_shape.h"
#include "phys/shape/concave_mesh_shape.h"

namespace pe_phys_collision {

    bool ConcaveCylinderCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                        pe::Transform trans_a, pe::Transform trans_b,
                                                        pe::Real refScale, ContactResult& result) {
        if (!((shape_a->getType() == pe_phys_shape::ShapeType::Cylinder &&
            shape_b->getType() == pe_phys_shape::ShapeType::ConcaveMesh) ||
            (shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh &&
                shape_b->getType() == pe_phys_shape::ShapeType::Cylinder))) {
            return false;
        }

        auto shape_concave = shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh ? shape_a : shape_b;
        auto shape_cyl = shape_a->getType() == pe_phys_shape::ShapeType::Box ? shape_a : shape_b;
        auto trans_concave = shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh ? trans_a : trans_b;
        auto trans_cyl = shape_a->getType() == pe_phys_shape::ShapeType::Box ? trans_a : trans_b;
        auto& mesh_concave = ((pe_phys_shape::ConcaveMeshShape*)shape_concave)->getMesh();
        auto& mesh_cyl = ((pe_phys_shape::CylinderShape*)shape_cyl)->getMesh();
        auto& edges_cyl = ((pe_phys_shape::CylinderShape*)shape_cyl)->getUniqueEdges();

        pe::Vector3 sep;
        pe::Real margin = PE_MARGIN;

        VertexArray world_verts_b1;
        VertexArray world_verts_b2;

        pe::Transform trans_cyl_rel2concave = trans_concave.inverse() * trans_cyl;
        pe::Vector3 convex_AA, convex_BB;
        shape_cyl->getAABB(trans_cyl_rel2concave, convex_AA, convex_BB);
        pe::Array<int> intersect;
        ((pe_phys_shape::ConcaveMeshShape*)shape_concave)->getIntersetFaces(convex_AA, convex_BB, intersect);

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
                shape_cyl, &shape_face, mesh_cyl, mesh_face,
                edges_cyl, shape_face.getUniqueEdges(),
                trans_cyl, trans_concave, sep, margin, result)) {
                continue;
            }
            ConvexConvexCollisionAlgorithm::clipHullAgainstHull(
                sep, mesh_cyl, mesh_face, trans_cyl, trans_concave,
                -refScale, 0, world_verts_b1, world_verts_b2,
                margin, result);
        }

        return true;
    }
    
} // pe_phys_collision