template <typename Scalar>
void Mesh<Scalar>::perFaceNormal(Mesh<Scalar>& mesh) {
    for (auto& face : mesh.faces) {
        if (face.indices.size() < 3) {
            throw std::runtime_error("Invalid mesh face: less than 3 vertices");
        }
        auto& p1 = mesh.vertices[face.indices[0]].position;
        auto& p2 = mesh.vertices[face.indices[1]].position;
        auto& p3 = mesh.vertices[face.indices[2]].position;
        face.normal = (p2 - p1).cross(p3 - p1).normalized();
    }
}

template <typename Scalar>
void Mesh<Scalar>::perVertexNormal(Mesh<Scalar>& mesh) {
    perFaceNormal(mesh);
    int v_n = (int)mesh.vertices.size();
    int f_n = (int)mesh.faces.size();
    std::vector<std::vector<int>> _map(v_n);
    for (int i = 0; i < f_n; i++) {
        for (uint32_t j : mesh.faces[i].indices) {
            _map[j].push_back(i);
        }
    }
    for (int i = 0; i < v_n; i++) {
        Vector3<Scalar> normal = Vector3<Scalar>::zeros();
        for (auto j : _map[i]) {
            normal += mesh.faces[j].normal;
        }
        normal /= (Scalar)_map[i].size();
        mesh.vertices[i].normal = normal.normalized();
    }
}
