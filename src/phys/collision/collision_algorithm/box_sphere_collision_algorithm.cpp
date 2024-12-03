#include "box_sphere_collision_algorithm.h"

// style-checked.
namespace pe_phys_collision {

    bool BoxSphereCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                       pe::Transform trans_a, pe::Transform trans_b,
                                                       pe::Real refScale, ContactResult& result) {
        if (!((shape_a->getType() == pe_phys_shape::ShapeType::ST_Box &&
               shape_b->getType() == pe_phys_shape::ShapeType::ST_Sphere) ||
              (shape_a->getType() == pe_phys_shape::ShapeType::ST_Sphere &&
               shape_b->getType() == pe_phys_shape::ShapeType::ST_Box))) {
            return false;
        }

        const auto shape_sph = dynamic_cast<pe_phys_shape::SphereShape *>(shape_a->getType() == pe_phys_shape::ShapeType::ST_Sphere ? shape_a : shape_b);
        const auto shape_box = dynamic_cast<pe_phys_shape::BoxShape *>(shape_a->getType() == pe_phys_shape::ShapeType::ST_Box ? shape_a : shape_b);
        const auto trans_sph = shape_a->getType() == pe_phys_shape::ShapeType::ST_Sphere ? trans_a : trans_b;
        const auto trans_box = shape_a->getType() == pe_phys_shape::ShapeType::ST_Box ? trans_a : trans_b;
        const pe::Vector3 center_sph = trans_sph.getOrigin();
        const pe::Real radius_sph = shape_sph->getRadius();
        constexpr auto margin = PE_MARGIN;

        result.setSwapFlag(shape_a->getType() == pe_phys_shape::ShapeType::ST_Box);
        bool ret = getClosestPoints(shape_sph, shape_box, trans_sph, trans_box, center_sph, radius_sph, margin, result);
        result.setSwapFlag(false);

        return ret;
    }

    static pe::Real getSpherePenetration(const pe::Vector3 &half_extent, const pe::Vector3 &pos_sph2box,
                                         pe::Vector3& closest_point, pe::Vector3& normal) {
        //project the center of the sphere on the closest face of the box
        pe::Real face_dist = half_extent.x() - pos_sph2box.x();
        pe::Real min_dist = face_dist;
        closest_point.x() = half_extent.x();
        normal = {1, 0, 0};

        face_dist = half_extent.x() + pos_sph2box.x();
        if (face_dist < min_dist) {
            min_dist = face_dist;
            closest_point = pos_sph2box;
            closest_point.x() = -half_extent.x();
            normal = {-1, 0, 0};
        }

        face_dist = half_extent.y() - pos_sph2box.y();
        if (face_dist < min_dist) {
            min_dist = face_dist;
            closest_point = pos_sph2box;
            closest_point.y() = half_extent.x();
            normal = {0, 1, 0};
        }

        face_dist = half_extent.y() + pos_sph2box.y();
        if (face_dist < min_dist) {
            min_dist = face_dist;
            closest_point = pos_sph2box;
            closest_point.y() = -half_extent.y();
            normal = {0, -1, 0};
        }

        face_dist = half_extent.z() - pos_sph2box.z();
        if (face_dist < min_dist) {
            min_dist = face_dist;
            closest_point = pos_sph2box;
            closest_point.z() = half_extent.z();
            normal = {0, 0, 1};
        }

        face_dist = half_extent.z() + pos_sph2box.z();
        if (face_dist < min_dist) {
            min_dist = face_dist;
            closest_point = pos_sph2box;
            closest_point.z() = -half_extent.z();
            normal = {0, 0, -1};
        }

        return min_dist;
    }

    static bool getSphereDistance(const pe_phys_shape::BoxShape* shape_box, const pe::Transform& trans_box,
                                  const pe::Vector3& center_sph, const pe::Real radius_sph,
                                  pe::Vector3& pt_on_box, pe::Vector3& normal, pe::Real& dist) {
        const pe::Vector3 boxHalfExtent = shape_box->getSize() / R(2.0);
        dist = R(1.0);

        // convert the sphere position to the box's local space
        const pe::Vector3 pos_sph2box = trans_box.inverseTransform(center_sph);

        // Determine the closest point to the sphere center in the box
        pe::Vector3 closest_point = pos_sph2box;
        closest_point.x() = PE_MIN(boxHalfExtent.x(), closest_point.x());
        closest_point.x() = PE_MAX(-boxHalfExtent.x(), closest_point.x());
        closest_point.y() = PE_MIN(boxHalfExtent.y(), closest_point.y());
        closest_point.y() = PE_MAX(-boxHalfExtent.y(), closest_point.y());
        closest_point.z() = PE_MIN(boxHalfExtent.z(), closest_point.z());
        closest_point.z() = PE_MAX(-boxHalfExtent.z(), closest_point.z());

        normal = pos_sph2box - closest_point;

        //if there is no penetration, we are done
        const pe::Real dist2 = normal.squaredNorm();
        if (dist2 > radius_sph * radius_sph) {
            return false;
        }

        pe::Real distance;

        //special case if the sphere center is inside the box
        if (dist2 <= PE_EPS * PE_EPS) {
            distance = -getSpherePenetration(boxHalfExtent, pos_sph2box, closest_point, normal);
        } else { //compute the penetration details
            distance = normal.norm();
            normal /= distance;
        }

        normal = trans_box.getBasis() * normal;
        pt_on_box = trans_box * closest_point;
        dist = distance - radius_sph;

        return true;
    }

    bool BoxSphereCollisionAlgorithm::getClosestPoints(pe_phys_shape::SphereShape *shape_sph, pe_phys_shape::BoxShape *shape_box,
                                                       const pe::Transform &trans_sph, const pe::Transform &trans_box,
                                                       const pe::Vector3 &center_sph, pe::Real radius_sph,
                                                       pe::Real margin, ContactResult &result) {
        pe::Vector3 pt_on_box, normal;
        pe::Real dist;
        if (getSphereDistance(shape_box, trans_box, center_sph, radius_sph,
                              pt_on_box, normal, dist)) {
            result.addContactPoint(normal, pt_on_box - normal * margin,
                                   dist + 2 * margin);
            result.setSwapFlag(false);
            return true;
        }

        return false;
    }


} // pe_phys_collision