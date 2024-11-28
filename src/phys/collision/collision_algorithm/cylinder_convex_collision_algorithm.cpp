#include "cylinder_convex_collision_algorithm.h"
#include "phys/shape/concave_mesh_shape.h"
#include "phys/shape/cylinder_shape.h"
#include "phys/shape/convex_mesh_shape.h"
#include "convex_convex_collision_algorithm.h"
#include "box_cylinder_collision_algorithm.h"

// cylinder-convex collision: refer to box-cylinder collision
namespace pe_phys_collision {

    bool CylinderConvexCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                            pe::Transform trans_a, pe::Transform trans_b,
                                                            pe::Real refScale, ContactResult& result) {
        if (!((shape_a->getType() == pe_phys_shape::ShapeType::ST_Cylinder &&
               shape_b->getType() == pe_phys_shape::ShapeType::ST_ConvexMesh) ||
              (shape_a->getType() == pe_phys_shape::ShapeType::ST_ConvexMesh &&
               shape_b->getType() == pe_phys_shape::ShapeType::ST_Cylinder))) {
            return false;
               }
        constexpr auto margin = PE_MARGIN;

#   if false
        auto& mesh_a = shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ?
                       dynamic_cast<pe_phys_shape::CylinderShape *>(shape_a)->getMesh() :
                       dynamic_cast<pe_phys_shape::ConvexMeshShape *>(shape_a)->getMesh();
        auto& mesh_b = shape_b->getType() == pe_phys_shape::ShapeType::Cylinder ?
                       dynamic_cast<pe_phys_shape::CylinderShape *>(shape_b)->getMesh() :
                       dynamic_cast<pe_phys_shape::ConvexMeshShape *>(shape_b)->getMesh();
        auto& edges_a = shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ?
                        dynamic_cast<pe_phys_shape::CylinderShape *>(shape_a)->getUniqueEdges() :
                        dynamic_cast<pe_phys_shape::ConvexMeshShape *>(shape_a)->getUniqueEdges();
        auto& edges_b = shape_b->getType() == pe_phys_shape::ShapeType::Cylinder ?
                        dynamic_cast<pe_phys_shape::CylinderShape *>(shape_b)->getUniqueEdges() :
                        dynamic_cast<pe_phys_shape::ConvexMeshShape *>(shape_b)->getUniqueEdges();

        return ConvexConvexCollisionAlgorithm::getClosestPoints(
            shape_a, shape_b, mesh_a, mesh_b, edges_a, edges_b,
            trans_a, trans_b, margin, refScale, result);
#   else
        auto shape_mesh = shape_a->getType() == pe_phys_shape::ShapeType::ST_ConvexMesh ? shape_a : shape_b;
        auto shape_cyl = dynamic_cast<pe_phys_shape::CylinderShape *>(shape_a->getType() == pe_phys_shape::ShapeType::ST_Cylinder ? shape_a : shape_b);
        auto& trans_cyl = shape_a->getType() == pe_phys_shape::ShapeType::ST_Cylinder ? trans_a : trans_b;
        auto& trans_mesh = shape_a->getType() == pe_phys_shape::ShapeType::ST_ConvexMesh ? trans_a : trans_b;

        result.setSwapFlag(shape_a->getType() == pe_phys_shape::ShapeType::ST_ConvexMesh);
        bool ret = getClosestPoints(shape_mesh, shape_cyl, trans_mesh, trans_cyl, margin, result);
        result.setSwapFlag(false);

        return ret;
#   endif
    }

    bool CylinderConvexCollisionAlgorithm::intersectSegmentFace(
            const pe::Mesh::Face& face, const pe::Mesh& mesh,
            const pe::Vector3& pos_seg, const pe::Vector3& dir_seg, pe::Real l_seg,
            pe::Real margin, pe::Real& t1, pe::Real& t2, pe::Real& d1, pe::Real& d2) {
        const pe::Vector3 origin = mesh.vertices[face.indices[0]].position;
        const pe::Vector3 end_seg = pos_seg + dir_seg * l_seg;
        const pe::Vector3 r1 = pos_seg - origin;
        const pe::Vector3 r2 = end_seg - origin;
        const pe::Real dist1 = r1.dot(face.normal);
        const pe::Real dist2 = r2.dot(face.normal);

        // both on the same side of the face plane
        if ((dist1 > 0 && dist2 > 0) || (dist1 <= 0 && dist2 <= 0)) {
            return false;
        }

        // one upside and one downside the face plane
        const pe::Real t = dist1 / (dist1 - dist2) * l_seg;
        const pe::Vector3 v = pos_seg + dir_seg * t;

        // check if the start or end point is inside the vertical face defined by the edge and the normal
        pe::Real t_min = 0, t_max = l_seg;
        auto min_dist_to_edge = PE_REAL_MAX;
        for (int i = 0; i < I(face.indices.size()); i++) {
            const pe::Vector3& v0 = mesh.vertices[face.indices[i]].position;
            const pe::Vector3& v1 = mesh.vertices[face.indices[(i + 1) % face.indices.size()]].position;
            const pe::Vector3 to_center = face.normal.cross(v1 - v0).normalized();
            const pe::Real dir_dot_to_center = dir_seg.dot(to_center);
            min_dist_to_edge = PE_MIN(min_dist_to_edge, PE_ABS((v0 - v).dot(to_center)));
            if (!PE_APPROX_EQUAL(dir_dot_to_center, 0)) {
                if (dir_dot_to_center > 0) {
                    t_min = PE_MAX(t_min, (v0 - pos_seg).dot(to_center) / dir_dot_to_center);
                } else {
                    t_max = PE_MIN(t_max, (v0 - pos_seg).dot(to_center) / dir_dot_to_center);
                }
            } else {
                if ((v0 - v).cross(v1 - v).dot(face.normal) < 0) {
                    return false;
                }
            }
        }

        if (t < t_min || t > t_max) {
            return false;
        }
        t1 = dist1 <= 0 ? t_min : t;
        t2 = dist2 <= 0 ? t_max : t;
        d1 = dist1 <= 0 ? dist1 * (t - t_min) / t : 0;
        d2 = dist2 <= 0 ? dist2 * (t_max - t) / (l_seg - t) : 0;
        // to prevent wrong contact points from another adjacent face
        if (-d1 > min_dist_to_edge + margin || -d2 > min_dist_to_edge + margin) {
            return false;
        }
        return true;
    }

    bool CylinderConvexCollisionAlgorithm::getClosestPoints(pe_phys_shape::Shape *shape_mesh, pe_phys_shape::CylinderShape *shape_cyl,
                                                            const pe::Transform &trans_mesh, const pe::Transform &trans_cyl,
                                                            pe::Real margin, ContactResult &result) {
        auto shape = shape_mesh->getType() == pe_phys_shape::ShapeType::ST_ConvexMesh ?
                     dynamic_cast<pe_phys_shape::ConvexMeshShape *>(shape_mesh) :
                     dynamic_cast<pe_phys_shape::ConcaveMeshShape *>(shape_mesh);
        auto& mesh = shape->getMesh();
        const pe::Real cyl_r = shape_cyl->getRadius();
        const pe::Real cyl_h = shape_cyl->getHeight() / R(2.0);
        pe::Transform trans_cyl2mesh = trans_mesh.inverse() * trans_cyl;
        pe::Vector3 axis_cyl = trans_cyl2mesh.getBasis().getColumn(1);
        pe::Vector3 pos_cyl = trans_cyl2mesh.getOrigin();

        pe::Vector3 cyl_AA, cyl_BB;
        shape_cyl->getAABB(trans_cyl2mesh, cyl_AA, cyl_BB);
        pe::Array<int> intersect;
        shape->getIntersectFaces(cyl_AA, cyl_BB, intersect);

#   define ADD_CONTACT_POINT_ON_MESH(face, mesh, pos, dir, l, margin, trans) \
        do { \
            pe::Real t1, t2, d1, d2; \
            if (intersectSegmentFace(face, mesh, pos, dir, l, margin, t1, t2, d1, d2)) { \
                pe::Vector3 p1 = trans * (pos_seg + dir_seg * t1 - f.normal * d1); \
                pe::Vector3 p2 = trans * (pos_seg + dir_seg * t2 - f.normal * d2); \
                pe::Vector3 n = trans.getBasis() * f.normal; \
                result.addContactPoint(n, p1 - n * margin, d1 + margin * 2); \
                result.addContactPoint(n, p2 - n * margin, d2 + margin * 2); \
            } \
        } while (0)

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
                ADD_CONTACT_POINT_ON_MESH(f, mesh, pos_seg, dir_seg, l_seg, margin, trans_mesh);
            }
            // to obtain stability, add two more contact points
            if (vertical_flag) {
                for (int i_dir = 0; i_dir < 3; i_dir += 2) {
                    const pe::Vector3 pos_seg = pos_cyl + d_z * dir_x[i_dir] * cyl_r + d_y * dir_y[i_dir] * cyl_h;
                    const pe::Vector3 dir_seg = d_y * -dir_x[i_dir];
                    const pe::Real l_seg = cyl_h * 2;
                    ADD_CONTACT_POINT_ON_MESH(f, mesh, pos_seg, dir_seg, l_seg, margin, trans_mesh);
                }
            }
        }

        // check whether there is an edge that intersects the cylinder
        for (const auto fi : intersect) {
            auto& f = mesh.faces[fi];
            for (int i = 0; i < I(f.indices.size()); i++) {
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

        return true;
    }


} // pe_phys_collision