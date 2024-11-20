#include "concave_cylinder_collision_algorithm.h"
#include "concave_convex_collision_algorithm.h"
#include "convex_convex_collision_algorithm.h"
#include "cylinder_convex_collision_algorithm.h"
#include "box_cylinder_collision_algorithm.h"
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
        const auto shape_mesh = dynamic_cast<pe_phys_shape::ConcaveMeshShape *>(shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh ? shape_a : shape_b);
        const auto shape_cyl = dynamic_cast<pe_phys_shape::CylinderShape *>(shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ? shape_a : shape_b);
        auto& trans_cyl = shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ? trans_a : trans_b;
        auto& trans_mesh = shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh ? trans_a : trans_b;

        auto& mesh = shape_mesh->getMesh();
        const pe::Real cyl_r = shape_cyl->getRadius();
        const pe::Real cyl_h = shape_cyl->getHeight() / pe::Real(2.0);
        pe::Transform trans_cyl2mesh = trans_mesh.inverse() * trans_cyl;
        pe::Vector3 axis_cyl = trans_cyl2mesh.getBasis().getColumn(1);
        pe::Vector3 pos_cyl = trans_cyl2mesh.getOrigin();

        pe::Vector3 cyl_AA, cyl_BB;
        shape_cyl->getAABB(trans_cyl2mesh, cyl_AA, cyl_BB);
        pe::Array<int> intersect;
        shape_mesh->getIntersectFaces(cyl_AA, cyl_BB, intersect);

        result.setSwapFlag(shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh);

        // check whether there is a face that supports the cylinder
        // consider the four directions of the cylinder
        static int dir_x[4] = {1, -1, -1, 1};
        static int dir_y[4] = {1, 1, -1, -1};
        for (const auto fi : intersect) {
            auto& f = mesh.faces[fi];

            pe::Real a_dot_c = axis_cyl.dot(f.normal);
            pe::Vector3 d_x, d_y = axis_cyl, d_z;
            bool vertical_flag = false;
            if (PE_APPROX_EQUAL(PE_ABS(a_dot_c), 1)) {
                d_z = axis_cyl.cross(pe::Vector3::right()).normalized();
                d_x = axis_cyl.cross(d_z).normalized();
                vertical_flag = true;
            } else {
                d_x = f.normal.cross(axis_cyl).cross(axis_cyl).normalized();
            }

            for (int i_dir = 0; i_dir < 4; i_dir++) {
                const pe::Vector3 pos_seg = pos_cyl + d_x * dir_x[i_dir] * cyl_r + d_y * dir_y[i_dir] * cyl_h;
                const pe::Vector3 dir_seg = (dir_x[i_dir] == dir_y[i_dir] ? d_y : d_x) * -dir_x[i_dir];
                const pe::Real l_seg = (dir_x[i_dir] == dir_y[i_dir] ? cyl_h : cyl_r) * 2;
                pe::Real t1, t2, d1, d2;
                if (CylinderConvexCollisionAlgorithm::intersectSegmentFace(f, mesh, pos_seg, dir_seg, l_seg, margin, t1, t2, d1, d2)) {
                    pe::Vector3 p1 = trans_mesh * (pos_seg + dir_seg * t1 - f.normal * d1);
                    pe::Vector3 p2 = trans_mesh * (pos_seg + dir_seg * t2 - f.normal * d2);
                    pe::Vector3 n = trans_mesh.getBasis() * f.normal;
                    result.addContactPoint(n, p1 - n * margin, d1 + margin * 2);
                    result.addContactPoint(n, p2 - n * margin, d2 + margin * 2);
                }
            }
            // to obtain stability, add two more contact points
            if (vertical_flag) {
                for (int i_dir = 0; i_dir < 3; i_dir += 2) {
                    const pe::Vector3 pos_seg = pos_cyl + d_z * dir_x[i_dir] * cyl_r + d_y * dir_y[i_dir] * cyl_h;
                    const pe::Vector3 dir_seg = d_y * -dir_x[i_dir];
                    const pe::Real l_seg = cyl_h * 2;
                    pe::Real t1, t2, d1, d2;
                    if (CylinderConvexCollisionAlgorithm::intersectSegmentFace(f, mesh, pos_seg, dir_seg, l_seg, margin, t1, t2, d1, d2)) {
                        pe::Vector3 p1 = trans_mesh * (pos_seg + dir_seg * t1 - f.normal * d1);
                        pe::Vector3 p2 = trans_mesh * (pos_seg + dir_seg * t2 - f.normal * d2);
                        pe::Vector3 n = trans_mesh.getBasis() * f.normal;
                        result.addContactPoint(n, p1 - n * margin, d1 + margin * 2);
                        result.addContactPoint(n, p2 - n * margin, d2 + margin * 2);
                    }
                }
            }
        }

        // check whether there is an edge that intersects the cylinder
        for (const auto fi : intersect) {
            auto& f = mesh.faces[fi];
            for (int i = 0; i < (int)f.indices.size(); i++) {
                const auto& v1 = mesh.vertices[f.indices[i]].position;
                const auto& v2 = mesh.vertices[f.indices[(i + 1) % f.indices.size()]].position;
                pe::Real t1, t2;
                pe::Vector3 start_seg = (v1 + v2) / 2;
                pe::Real l_seg = (v2 - v1).norm() / 2;
                pe::Vector3 dir_seg = (v2 - v1) / (l_seg * 2);
                if (BoxCylinderCollisionAlgorithm::intersectSegmentCylinder(start_seg, dir_seg, l_seg, pos_cyl, axis_cyl, cyl_h, cyl_r, margin, t1, t2)) {
                    pe::Vector3 p1 = start_seg + dir_seg * t1;
                    pe::Vector3 p2 = start_seg + dir_seg * t2;
                    pe::Vector3 p_mid = (p1 + p2) / 2;
                    BoxCylinderCollisionAlgorithm::addContactPointOnCylinder(p_mid, pos_cyl, axis_cyl, cyl_r, cyl_h, trans_mesh, margin, result);
                }
            }
        }

        result.setSwapFlag(false);
        return true;
#   endif
    }
    
} // pe_phys_collision