#include "collision_algorithm.h"

namespace pe_phys_collision {

    bool CollisionAlgorithm::isInsideTriangle(const pe::Array<pe::Vector3> &triangle, const pe::Vector3 &normal,
                                              const pe::Vector3 &point, pe::Real tolerance) {
        // distance to plane
        pe::Real dist = point.dot(normal) - triangle[0].dot(normal);
        if (dist < -tolerance || dist > tolerance) return false;

        // TODO: check error tolerance
        // inside check
        pe::Vector3 proj = point - normal * dist;
        pe::Vector3 p1 = triangle[0];
        pe::Vector3 p2 = triangle[1];
        pe::Vector3 p3 = triangle[2];
        pe::Vector3 c1 = (p1 - proj).cross(p2 - proj);
        pe::Vector3 c2 = (p2 - proj).cross(p3 - proj);
        pe::Vector3 c3 = (p3 - proj).cross(p1 - proj);
        return c1.dot(c2) >= 0 && c2.dot(c3) >= 0;
    }

    void CollisionAlgorithm::calcTriangleAABB(const pe::Array<pe::Vector3> &triangle, pe::Vector3 &min,
                                              pe::Vector3 &max) {
        for (auto& tri : triangle) {
            min = PE_MIN_VEC(min, tri);
            max = PE_MAX_VEC(max, tri);
        }
    }

    bool CollisionAlgorithm::isSeparateOnAxis(pe_phys_shape::Shape *shape_a, pe_phys_shape::Shape *shape_b,
                                              const pe::Transform &transform_a, const pe::Transform &transform_b,
                                              pe::Vector3 &axis, pe::Real &depth,
                                              pe::Vector3 &contact_point_a, pe::Vector3 &contact_point_b) {
        pe::Real min0 = 0, max0 = 0;
        pe::Real min1 = 0, max1 = 0;
        pe::Vector3 point_min0, point_max0;
        pe::Vector3 point_min1, point_max1;

        // project shape_a and shape_b onto axis
        shape_a->project(transform_a, axis, min0, max0, point_min0, point_max0);
        shape_b->project(transform_b, axis, min1, max1, point_min1, point_max1);
        if (max0 < min1 || max1 < min0) return true;

        // TODO: check why using point of only one side
        // calculate the contact information
        pe::Real d0 = max0 - min1, d1 = max1 - min0;
        if (d0 < d1) {
            depth = d0;
            axis = -axis;
            contact_point_a = point_max0;
            contact_point_b = point_min1;
        } else {
            depth = d1;
            contact_point_a = point_min0;
            contact_point_b = point_max1;
        }
        return false;
    }

    bool CollisionAlgorithm::findSeparateAxis(pe_phys_shape::Shape *shape_a, pe_phys_shape::Shape *shape_b,
                                              const pe::Transform &transform_a, const pe::Transform &transform_b,
                                              const pe::Vector3 &overlapMin, const pe::Vector3 &overlapMax,
                                              pe::Vector3 &axis, ContactResult &result) {
        // TODO: learn the algorithm and check the correctness
    }

} // pe_phys_collision