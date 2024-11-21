#include "concave_convex_collision_algorithm.h"
#include "phys/shape/concave_mesh_shape.h"
#include "phys/shape/convex_mesh_shape.h"
#include "convex_convex_collision_algorithm.h"

// style-checked.
namespace pe_phys_collision {

    bool ConcaveConvexCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                           pe::Transform trans_a, pe::Transform trans_b,
                                                           pe::Real refScale, ContactResult& result) {
        if (!((shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh &&
            shape_b->getType() == pe_phys_shape::ShapeType::ConcaveMesh) ||
            (shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh &&
                shape_b->getType() == pe_phys_shape::ShapeType::ConvexMesh))) {
            return false;
        }

        auto shape_concave = dynamic_cast<pe_phys_shape::ConcaveMeshShape *>(shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh ? shape_a : shape_b);
        auto shape_convex = dynamic_cast<pe_phys_shape::ConvexMeshShape *>(shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh ? shape_a : shape_b);
        auto trans_concave = shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh ? trans_a : trans_b;
        auto trans_convex = shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh ? trans_a : trans_b;
        auto& mesh_concave = shape_concave->getMesh();
        auto& mesh_convex = shape_convex->getMesh();
        auto& edges_convex = shape_convex->getUniqueEdges();

        constexpr auto margin = PE_MARGIN;

        result.setSwapFlag(shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh);
        bool ret = getClosestPoints(
            shape_concave, shape_convex, trans_concave, trans_convex, edges_convex,
            mesh_concave, mesh_convex, margin, refScale, result);
        result.setSwapFlag(false);

        return ret;
    }

    bool ConcaveConvexCollisionAlgorithm::getClosestPoints(
            pe_phys_shape::ConcaveMeshShape *shape_concave, pe_phys_shape::Shape *shape_convex,
            const pe::Transform &trans_concave, const pe::Transform &trans_convex,
            const pe::Array<pe::KV<pe::Vector3, pe::Vector3>>& unique_edges_convex,
            const pe::Mesh &mesh_concave, const pe::Mesh &mesh_convex,
            pe::Real margin, pe::Real refScale, ContactResult &result) {
        pe::Transform trans_convex_rel2concave = trans_concave.inverse() * trans_convex;
        pe::Vector3 convex_AA, convex_BB;
        shape_convex->getAABB(trans_convex_rel2concave, convex_AA, convex_BB);
        pe::Array<int> intersect;
        shape_concave->getIntersectFaces(convex_AA, convex_BB, intersect);

        for (auto fi : intersect) {
            auto& f = mesh_concave.faces[fi];
            pe::Mesh mesh_face;
            pe::Mesh::Face face_face;
            for (int i = 0; i < I(f.indices.size()); i++) {
                mesh_face.vertices.push_back(mesh_concave.vertices[f.indices[i]]);
                face_face.indices.push_back(i);
            }
            face_face.normal = f.normal;
            mesh_face.faces.push_back(face_face);
            pe_phys_shape::ConvexMeshShape shape_face;
            shape_face.setMesh(mesh_face);

            ConvexConvexCollisionAlgorithm::getClosestPoints(
                shape_convex, &shape_face, mesh_convex, mesh_face,
                unique_edges_convex, shape_face.getUniqueEdges(),
                trans_convex, trans_concave, margin, refScale, result);
        }
        result.setSwapFlag(false);

        return true;
    }


} // pe_phys_collision