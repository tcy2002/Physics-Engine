#pragma once

#include "cloth/object/cloth_object.h"

namespace pe_phys_object {

    class PBDCloth : public ClothObject {
        COMMON_MEMBER_SET_GET(pe::Real, damping, Damping)
        COMMON_MEMBER_SET_GET(pe::Real, mass, Mass)
        COMMON_MEMBER_SET_GET(pe::Real, thickness, Thickness)
        COMMON_BOOL_SET_GET(enable_self_collision, EnableSelfCollision)
        COMMON_MEMBER_SET_GET(pe::Real, self_collision_distance, SelfCollisionDistance)
        COMMON_BOOL_SET_GET(enable_bending, EnableBending)
        COMMON_MEMBER_SET_GET(pe::Real, accuracy, Accuracy)
        COMMON_MEMBER_SET_GET(int, max_iterations, MaxIterations)
        COMMON_MEMBER_SET_GET(pe::Real, eps, Eps)
        COMMON_MEMBER_SET_GET(pe::Vector3, gravity, Gravity)
        COMMON_MEMBER_SET_GET(pe::Vector3, boundaryAA, BoundaryAA)
        COMMON_MEMBER_SET_GET(pe::Vector3, boundaryBB, BoundaryBB)
        COMMON_MEMBER_SET_GET(pe::Real, fixed_plane, FixedPlane)

    public:
        PE_API PBDCloth(const std::string& name = "PBDCloth", pe::Real stiffness = 1.0, pe::Real bending = 0.1);
        virtual ~PBDCloth() {}

        virtual void step(pe::Real dt) override;

    protected:
        // mesh properties
        pe::Map<std::pair<int, int>, pe::Real> _edges;
        pe::Real _max_tri_size;
        pe::Real _min_tri_size;
        pe::Real _avg_tri_size;
        void readMeshStructure();

        // fixed mesh vertices
        pe::Set<int> _fixed_verts;

        // simulation daeta
        pe::Array<pe::Vector3> _verts_orig;
        pe::Array<pe::Vector3> _verts_new;
        pe::Array<pe::Real> _inv_mass;
        pe::Array<pe::Vector3> _vels;

        // simulation boundary
        pe::Real project2Boundary(pe::Vector3& v);

        // for self-collision
        std::unordered_multimap<uint32_t, int> _hash2tri;
        std::unordered_multimap<int, uint32_t> _tri2hash;
        pe::Array<bool> _tri_should_update_hash;
        pe::Array<pe::Set<int>> _vert_ignore_tris;
        uint32_t getVertHashFromPos(const pe::Vector3 v);
        uint32_t getVertHashFromGrid(const int vx, int vy, int vz);
        void getVertGridFromPos(const pe::Vector3& v, int& vx, int& vy, int& vz);
        void getVertGridFromHash(uint32_t hash, int& vx, int& vy, int& vz);
        void updateTriangleHash(int tri);
        void updateTriangleHash(const pe::Array<pe::Vector3>& last_verts, const pe::Array<pe::Vector3>& new_verts);

        // for bending
        pe::Map<std::pair<int, int>, pe::Array<int>> _edge2verts;
        pe::Map<std::pair<int, int>, std::pair<pe::Real, int>> _edge_angle;

        // PBD
        void predictVelocityPosition(pe::Real dt);
        void projectPosition(pe::Real dt);

        // collision
        pe::Array<std::pair<int, int>> _self_collision_pairs;
        void findSelfCollisionPairs(int vert);
    };

} // namespace pe_phys_object