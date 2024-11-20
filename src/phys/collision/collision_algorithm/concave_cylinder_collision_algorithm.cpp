#include "concave_cylinder_collision_algorithm.h"
#include "concave_convex_collision_algorithm.h"
#include "convex_convex_collision_algorithm.h"
#include "cylinder_convex_collision_algorithm.h"
#include "phys/shape/cylinder_shape.h"
#include "phys/shape/concave_mesh_shape.h"

// style-checked.
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
        constexpr auto margin = PE_MARGIN;

#   if false
        auto shape_concave = dynamic_cast<pe_phys_shape::ConcaveMeshShape *>(shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh ? shape_a : shape_b);
        auto shape_cyl = dynamic_cast<pe_phys_shape::CylinderShape *>(shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ? shape_a : shape_b);
        auto trans_concave = shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh ? trans_a : trans_b;
        auto trans_cyl = shape_a->getType() == pe_phys_shape::ShapeType::Box ? trans_a : trans_b;
        auto& mesh_concave = shape_concave->getMesh();
        auto& mesh_cyl = shape_cyl->getMesh();
        auto& edges_cyl = shape_cyl->getUniqueEdges();

        pe::Vector3 sep;
        VertexArray world_vertices_b1;
        VertexArray world_vertices_b2;

        pe::Transform trans_cyl_rel2concave = trans_concave.inverse() * trans_cyl;
        pe::Vector3 convex_AA, convex_BB;
        shape_cyl->getAABB(trans_cyl_rel2concave, convex_AA, convex_BB);
        pe::Array<int> intersect;
        shape_concave->getIntersectFaces(convex_AA, convex_BB, intersect);

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
            pe_phys_shape::ConcaveMeshShape shape_face;
            shape_face._mesh = mesh_face;

            if (!ConvexConvexCollisionAlgorithm::findSeparatingAxis(
                shape_cyl, &shape_face, mesh_cyl, mesh_face,
                edges_cyl, shape_face.getUniqueEdges(),
                trans_cyl, trans_concave, sep, margin, result)) {
                continue;
            }
            ConvexConvexCollisionAlgorithm::clipHullAgainstHull(
                sep, mesh_cyl, mesh_face, trans_cyl, trans_concave,
                -refScale, margin, world_vertices_b1, world_vertices_b2,
                margin, result);
        }
        result.setSwapFlag(false);
        return true;
#   else
        auto shape_mesh = (pe_phys_shape::ConcaveMeshShape*)(shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh ? shape_a : shape_b);
        auto shape_cyl = (pe_phys_shape::CylinderShape*)(shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ? shape_a : shape_b);
        auto& mesh = shape_mesh->getMesh();
        auto& trans_mesh = shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh ? trans_a : trans_b;
        pe::Real cyl_r = shape_cyl->getRadius();
        pe::Real cyl_h = shape_cyl->getHeight() / pe::Real(2.0);
        auto& trans_cyl = shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ? trans_a : trans_b;

        pe::Transform trans_mesh2cyl = trans_cyl.inverse() * trans_mesh;
        pe::Transform trans_cyl2mesh = trans_mesh2cyl.inverse();
        pe::Vector3 cyl_AA, cyl_BB;
        shape_cyl->getAABB(trans_cyl2mesh, cyl_AA, cyl_BB);
        pe::Array<int> intersect;
        shape_mesh->getIntersectFaces(cyl_AA, cyl_BB, intersect);

        result.setSwapFlag(shape_b->getType() == pe_phys_shape::ShapeType::ConcaveMesh);
        for (auto fi : intersect) {
            auto& f = mesh.faces[fi];
            pe::Array<pe::Vector3> vertices;
            vertices.reserve(f.indices.size());
            for (auto vi : f.indices) {
                vertices.push_back(trans_mesh2cyl * mesh.vertices[vi].position);
            }
            pe::Vector3 n = trans_mesh2cyl.getBasis() * f.normal;

            pe::Array<bool> point_inside(f.indices.size(), false);
            bool non_point_inside = true;
            for (int i = 0; i < (int)f.indices.size(); i++) {
                point_inside[i] = CylinderConvexCollisionAlgorithm::pointInsideCylinder(vertices[i], cyl_h, cyl_r, trans_cyl, margin, result);
                non_point_inside = non_point_inside && !point_inside[i];
            }

            for (int i = 0; i < (int)f.indices.size(); i++) {
                int v1 = i;
                int v2 = (i + 1) % (int)f.indices.size();
                if (!point_inside[v1] && !point_inside[v2]) {
                    CylinderConvexCollisionAlgorithm::intersectSegmentCylinder(vertices[v1], vertices[v2], cyl_h, cyl_r, trans_cyl, margin, result);
                }
            }

            if (non_point_inside) {
                CylinderConvexCollisionAlgorithm::intersectFaceCylinder(vertices, n, cyl_h, cyl_r, trans_cyl, margin, result);
            }
        }
        result.setSwapFlag(false);
        return true;
#   endif
    }
    
} // pe_phys_collision