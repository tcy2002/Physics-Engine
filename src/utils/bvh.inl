#pragma once

template <typename Scalar>
BVH<Scalar>::BVH(common::Mesh<Scalar>& mesh, int max_node) {
    bvh = buildBVH(mesh, 0, (int)mesh.faces.size() - 1, max_node);
}

template <typename Scalar>
common::Vector3<Scalar> min_vector3(const common::Mesh<Scalar>& mesh, const typename common::Mesh<Scalar>::Face& face) {
    Scalar min_x = PE_BVH_MAX, min_y = PE_BVH_MAX, min_z = PE_BVH_MAX;
    for (unsigned int index : face.indices) {
        auto& pos = mesh.vertices[index].position;
        if (pos.x < min_x) min_x = pos.x;
        if (pos.y < min_y) min_y = pos.y;
        if (pos.z < min_z) min_z = pos.z;
    }
    return { min_x, min_y, min_z };
}

template <typename Scalar>
common::Vector3<Scalar> max_vector3(const common::Mesh<Scalar>& mesh, const typename common::Mesh<Scalar>::Face& face) {
    Scalar max_x = PE_BVH_MIN, max_y = PE_BVH_MIN, max_z = PE_BVH_MIN;
    for (unsigned int index : face.indices) {
        auto& pos = mesh.vertices[index].position;
        if (pos.x > max_x) max_x = pos.x;
        if (pos.y > max_y) max_y = pos.y;
        if (pos.z > max_z) max_z = pos.z;
    }
    return { max_x, max_y, max_z };
}

template <typename Scalar>
bool cmp(const common::Mesh<Scalar>& mesh, const typename common::Mesh<Scalar>::Face& a, const typename common::Mesh<Scalar>::Face& b, int axis) {
    Scalar sum_a = 0, sum_b = 0;
    for (unsigned int index : a.indices) {
        sum_a += mesh.vertices[index].position[axis];
    }
    sum_a /= (Scalar)a.indices.size();
    for (unsigned int index : b.indices) {
        sum_b += mesh.vertices[index].position[axis];
    }
    sum_b /= (Scalar)b.indices.size();
    return sum_a < sum_b;
}

template <typename Scalar>
BVHNode<Scalar>* BVH<Scalar>::buildBVH(common::Mesh<Scalar>& mesh, int l, int r, int n) {
    if (l > r) return nullptr;
    auto node = new BVHNode<Scalar>();

    for (int i = l; i <= r; i++) {
        common::Vector3<Scalar> tmp = min_vector3(mesh, mesh.faces[i]);
        if (tmp.x < node->AA.x) node->AA.x = tmp.x;
        if (tmp.y < node->AA.y) node->AA.y = tmp.y;
        if (tmp.z < node->AA.z) node->AA.z = tmp.z;
        tmp = max_vector3(mesh, mesh.faces[i]);
        if (tmp.x > node->BB.x) node->BB.x = tmp.x;
        if (tmp.y > node->BB.y) node->BB.y = tmp.y;
        if (tmp.z > node->BB.z) node->BB.z = tmp.z;
    }

    if (r - l + 1 <= n) {
        node->n = r - l + 1;
        node->index = l;
        return node;
    }

    Scalar len_x = node->BB.x - node->AA.x;
    Scalar len_y = node->BB.y - node->AA.y;
    Scalar len_z = node->BB.z - node->AA.z;

    if (len_x >= len_y && len_x >= len_z)
        std::sort(mesh.faces.begin() + l, mesh.faces.begin() + r + 1, [this, &mesh](const typename common::Mesh<Scalar>::Face& a, const typename common::Mesh<Scalar>::Face& b) { return cmp(mesh, a, b, 0); });
    else if (len_y >= len_z)
        std::sort(mesh.faces.begin() + l, mesh.faces.begin() + r + 1, [this, &mesh](const typename common::Mesh<Scalar>::Face& a, const typename common::Mesh<Scalar>::Face& b) { return cmp(mesh, a, b, 1); });
    else
        std::sort(mesh.faces.begin() + l, mesh.faces.begin() + r + 1, [this, &mesh](const typename common::Mesh<Scalar>::Face& a, const typename common::Mesh<Scalar>::Face& b) { return cmp(mesh, a, b, 2); });

    int mid = (l + r) / 2;
    node->left = buildBVH(mesh, l, mid, n);
    node->right = buildBVH(mesh, mid + 1, r, n);
    if (node->left != nullptr) node->size += node->left->size;
    if (node->right != nullptr) node->size += node->right->size;
    return node;
}

template <typename Scalar>
void BVH<Scalar>::clear() {
    if (bvh != nullptr) {
        std::vector<BVHNode<Scalar>*> stack;
        stack.push_back(bvh);
        while (!stack.empty()) {
            auto node = stack.back();
            stack.pop_back();
            if (node == nullptr) continue;
            stack.push_back(node->left);
            stack.push_back(node->right);
            delete node;
        }
    }
}

template <typename Scalar>
void BVH<Scalar>::setMesh(common::Mesh<Scalar>& mesh, int max_node) {
    if (bvh != nullptr) clear();
    bvh = buildBVH(mesh, 0, static_cast<int>(mesh.faces.size()) - 1, max_node);
}

template <typename Scalar>
void BVH<Scalar>::search(const common::Vector3<Scalar>& AA, const common::Vector3<Scalar>& BB, std::vector<int>& intersect) const {
    if (bvh == nullptr) return;
    std::vector<BVHNode<Scalar>*> stack;
    stack.push_back(bvh);
    while (!stack.empty()) {
        auto node = stack.back();
        stack.pop_back();
        if (node == nullptr) continue;
        if (node->AA.x > BB.x || node->AA.y > BB.y || node->AA.z > BB.z) continue;
        if (node->BB.x < AA.x || node->BB.y < AA.y || node->BB.z < AA.z) continue;
        if (node->n > 0) {
            for (int i = 0; i < node->n; i++) {
                intersect.push_back(node->index + i);
            }
        }
        else {
            stack.push_back(node->left);
            stack.push_back(node->right);
        }
    }
}
