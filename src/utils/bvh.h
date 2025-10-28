#pragma once

#include <common/vector3.h>
#include <common/matrix3x3.h>
#include <common/mesh.h>
#include <vector>

#ifdef PE_USE_DOUBLE
#define BVH_REAL double
#define BVH_MAX BVH_REAL(1.0e+300)
#define BVH_MIN BVH_REAL(-1.0e+300)
#else
#define BVH_REAL float
#define BVH_MAX BVH_REAL(1.0e+30)
#define BVH_MIN BVH_REAL(-1.0e+30)
#endif

namespace utils {

    using Vector3 = common::Vector3<BVH_REAL>;
    using Mesh = common::Mesh<BVH_REAL>;

    struct BVHNode {
        BVHNode* left = nullptr;
        BVHNode* right = nullptr;
        int size = 1;
        int n = 0;
        int index = 0;
        Vector3 AA = { BVH_MAX, BVH_MAX, BVH_MAX };
        Vector3 BB = { BVH_MIN, BVH_MIN, BVH_MIN };
    };

    class BVH {
    private:
        BVHNode* bvh;
        BVHNode* buildBVH(Mesh& mesh, int l, int r, int n);
        void clear();

    public:
        BVH() : bvh(nullptr) {}
        BVH(Mesh& mesh, int max_node);
        ~BVH() { clear(); }
        void setMesh(Mesh& mesh, int max_node);
        void search(const Vector3& AA, const Vector3& BB, std::vector<int>& intersect) const;
    };

} // namespace utils