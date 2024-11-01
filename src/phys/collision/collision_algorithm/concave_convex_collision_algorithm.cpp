#include "concave_convex_collision_algorithm.h"
#include <phys/shape/concave_mesh_shape.h>
#include <phys/shape/convex_mesh_shape.h>
#include "convex_convex_collision_algorithm.h"

namespace pe_phys_collision {

    bool ConcaveConvexCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                           pe::Transform trans_a, pe::Transform trans_b,
                                                           ContactResult& result) {
        if (!((shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh &&
            shape_b->getType() == pe_phys_shape::ShapeType::ConcaveMesh) ||
            (shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh &&
                shape_b->getType() == pe_phys_shape::ShapeType::ConvexMesh))) {
            return false;
        }

        auto shape_concave = shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh ? shape_a : shape_b;
        auto shape_convex = shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh ? shape_a : shape_b;
        auto trans_concave = shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh ? trans_a : trans_b;
        auto trans_convex = shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh ? trans_a : trans_b;
        auto& mesh_concave = ((pe_phys_shape::ConcaveMeshShape*)shape_concave)->getMesh();
        auto& mesh_convex = ((pe_phys_shape::ConvexMeshShape*)shape_convex)->getMesh();

        pe::Vector3 sep;
        pe::Real margin = PE_MARGIN;

        VertexArray world_verts_b1;
        VertexArray world_verts_b2;

        pe::Transform trans_convex_rel2concave = trans_concave.inverse() * trans_convex;
        pe::Vector3 convex_AA, convex_BB;
        shape_convex->getAABB(trans_convex_rel2concave, convex_AA, convex_BB);
        pe::Array<int> intersect;
        ((pe_phys_shape::ConvexMeshShape*)shape_concave)->getIntersetFaces(convex_AA, convex_BB, intersect);

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
            shape_face.setMesh(mesh_face);

            if (!ConvexConvexCollisionAlgorithm::findSeparatingAxis(
                shape_convex, &shape_face, mesh_convex, mesh_face,
                ((pe_phys_shape::ConvexMeshShape*)shape_convex)->getUniqueEdges(),
                shape_face.getUniqueEdges(),
                trans_convex, trans_concave, sep, margin, result)) {
                continue;
                }
            ConvexConvexCollisionAlgorithm::clipHullAgainstHull(
                sep, mesh_convex, mesh_face, trans_convex, trans_concave,
                PE_REAL_MIN, margin, world_verts_b1, world_verts_b2,
                margin, result);
        }

        return true;
    }

} // pe_phys_collision