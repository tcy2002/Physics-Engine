#include "box_sphere_collision_algorithm.h"

namespace pe_phys_collision {

    bool BoxSphereCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                       pe::Transform trans_a, pe::Transform trans_b,
                                                       pe::Real refScale, ContactResult& result) {
        if (!((shape_a->getType() == pe_phys_shape::ShapeType::Box &&
               shape_b->getType() == pe_phys_shape::ShapeType::Sphere) ||
              (shape_a->getType() == pe_phys_shape::ShapeType::Sphere &&
               shape_b->getType() == pe_phys_shape::ShapeType::Box))) {
            return false;
        }

        auto shape_sph = (pe_phys_shape::SphereShape*)(shape_a->getType() == pe_phys_shape::ShapeType::Sphere ? shape_a : shape_b);
        auto shape_box = (pe_phys_shape::BoxShape*)(shape_a->getType() == pe_phys_shape::ShapeType::Box ? shape_a : shape_b);
        auto trans_sph = shape_a->getType() == pe_phys_shape::ShapeType::Sphere ? trans_a : trans_b;
        auto trans_box = shape_a->getType() == pe_phys_shape::ShapeType::Box ? trans_a : trans_b;
        pe::Vector3 sphereCenter = trans_sph.getOrigin();
        pe::Real radius = shape_sph->getRadius();
        pe::Real margin = PE_MARGIN;

        pe::Vector3 ptOnBox, normal;
        pe::Real dist;
        if (getSphereDistance(shape_box, trans_box, sphereCenter, radius,
                              ptOnBox, normal, dist)) {
            result.setSwapFlag(shape_a->getType() == pe_phys_shape::ShapeType::Box);
            result.addContactPoint(normal, ptOnBox - normal * margin,
                                   dist + 2 * margin);
            return true;
        }

        return false;
    }

    bool BoxSphereCollisionAlgorithm::getSphereDistance(const pe_phys_shape::BoxShape *boxShape,
                                                        const pe::Transform& boxTrans,
                                                        const pe::Vector3& sphereCenter, pe::Real radius,
                                                        pe::Vector3 &ptOnBox, pe::Vector3 &normal, pe::Real &dist) {
        pe::Vector3 const& boxHalfExtent = boxShape->getSize() / 2.0;
        dist = 1.0f;

        // convert the sphere position to the box's local space
        pe::Vector3 sphereRelPos = boxTrans.inverseTransform(sphereCenter);

        // Determine the closest point to the sphere center in the box
        pe::Vector3 closestPoint = sphereRelPos;
        closestPoint.x = PE_MIN(boxHalfExtent.x, closestPoint.x);
        closestPoint.x = PE_MAX(-boxHalfExtent.x, closestPoint.x);
        closestPoint.y = PE_MIN(boxHalfExtent.y, closestPoint.y);
        closestPoint.y = PE_MAX(-boxHalfExtent.y, closestPoint.y);
        closestPoint.z = PE_MIN(boxHalfExtent.z, closestPoint.z);
        closestPoint.z = PE_MAX(-boxHalfExtent.z, closestPoint.z);

        normal = sphereRelPos - closestPoint;

        //if there is no penetration, we are done
        pe::Real dist2 = normal.norm2();
        if (dist2 > radius * radius) {
            return false;
        }

        pe::Real distance;

        //special case if the sphere center is inside the box
        if (dist2 <= PE_EPS * PE_EPS) {
            distance = -getSpherePenetration(boxHalfExtent, sphereRelPos, closestPoint, normal);
        } else { //compute the penetration details
            distance = normal.norm();
            normal /= distance;
        }

        normal = boxTrans.getBasis() * normal;
        ptOnBox = boxTrans * closestPoint;
        dist = distance - radius;

        return true;
    }

    pe::Real BoxSphereCollisionAlgorithm::getSpherePenetration(const pe::Vector3 &boxHalfExt,
                                                               const pe::Vector3 &sphereRelPos,
                                                               pe::Vector3 &closestPoint, pe::Vector3 &normal) {
        //project the center of the sphere on the closest face of the box
        pe::Real faceDist = boxHalfExt.x - sphereRelPos.x;
        pe::Real minDist = faceDist;
        closestPoint.x = boxHalfExt.x;
        normal = {1, 0, 0};

        faceDist = boxHalfExt.x + sphereRelPos.x;
        if (faceDist < minDist) {
            minDist = faceDist;
            closestPoint = sphereRelPos;
            closestPoint.x = -boxHalfExt.x;
            normal = {-1, 0, 0};
        }

        faceDist = boxHalfExt.y - sphereRelPos.y;
        if (faceDist < minDist) {
            minDist = faceDist;
            closestPoint = sphereRelPos;
            closestPoint.y = boxHalfExt.x;
            normal = {0, 1, 0};
        }

        faceDist = boxHalfExt.y + sphereRelPos.y;
        if (faceDist < minDist) {
            minDist = faceDist;
            closestPoint = sphereRelPos;
            closestPoint.y = -boxHalfExt.y;
            normal = {0, -1, 0};
        }

        faceDist = boxHalfExt.z - sphereRelPos.z;
        if (faceDist < minDist) {
            minDist = faceDist;
            closestPoint = sphereRelPos;
            closestPoint.z = boxHalfExt.z;
            normal = {0, 0, 1};
        }

        faceDist = boxHalfExt.z + sphereRelPos.z;
        if (faceDist < minDist) {
            minDist = faceDist;
            closestPoint = sphereRelPos;
            closestPoint.z = -boxHalfExt.z;
            normal = {0, 0, -1};
        }

        return minDist;
    }

} // pe_phys_collision