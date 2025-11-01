#pragma once

#include <common/vector3.h>
#include <common/mesh.h>
#include <vector>
#include <algorithm>

namespace pe_utils {

#define PE_BVH_MAX 1e30
#define PE_BVH_MIN (-1e30)

template <typename Scalar>
struct BVHNode {
    BVHNode* left = nullptr;
    BVHNode* right = nullptr;
    int size = 1;
    int n = 0;
    int index = 0;
    common::Vector3<Scalar> AA = { PE_BVH_MAX, PE_BVH_MAX, PE_BVH_MAX };
    common::Vector3<Scalar> BB = { PE_BVH_MIN, PE_BVH_MIN, PE_BVH_MIN };
};

template <typename Scalar>
class BVH {
private:
    BVHNode<Scalar>* bvh;
    BVHNode<Scalar>* buildBVH(common::Mesh<Scalar>& mesh, int l, int r, int n);
    void clear();

public:
    BVH() : bvh(nullptr) {}
    BVH(common::Mesh<Scalar>& mesh, int max_node);
    ~BVH() { clear(); }
    void setMesh(common::Mesh<Scalar>& mesh, int max_node);
    void search(const common::Vector3<Scalar>& AA, const common::Vector3<Scalar>& BB, std::vector<int>& intersect) const;
};

#include "bvh.inl"

} // namespace pe_utils