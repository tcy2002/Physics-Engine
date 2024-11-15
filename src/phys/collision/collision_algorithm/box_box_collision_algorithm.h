#pragma once

#include "collision_algorithm.h"
#include "phys/shape/box_shape.h"

namespace pe_phys_collision {

    typedef pe::Real dMatrix3[4 * 3];

    class BoxBoxCollisionAlgorithm : public CollisionAlgorithm {
    public:
        virtual bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                      pe::Transform trans_a, pe::Transform trans_b,
                                      pe::Real refScale, ContactResult& result) override;

    private:
        static int intersectRectQuad2(const pe::Real h[2], pe::Real p[8], pe::Real ret[16]);
        static void cullPoints2(int n, pe::Real p[], int m, int i0, int i_ret[]);
    };

} // pe_phys_collision