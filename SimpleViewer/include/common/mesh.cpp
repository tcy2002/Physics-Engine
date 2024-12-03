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
        Vector3<Scalar> normal = Vector3<Scalar>::Zero();
        for (auto j : _map[i]) {
            normal += mesh.faces[j].normal;
        }
        normal /= (Scalar)_map[i].size();
        mesh.vertices[i].normal = normal.normalized();
    }
}

template<typename Scalar>
void Mesh<Scalar>::loadFromObj(const std::string &filename, Mesh<Scalar> &mesh, const Vector3<Scalar>& size) {
    std::fstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file." << filename << std::endl;
        return;
    }

    char buf[1024];
    while (file.getline(buf, 1024)) {
        std::stringstream ss(buf);
        std::string str;
        ss >> str;
        if (str == "v") {
            Scalar x, y, z;
            ss >> x >> y >> z;
            mesh.vertices.push_back({{x * size.x(), y * size.x(), z * size.x()}, {0, 0, 0}});
        }
        else if (str == "f") {
            std::string vert;
            Mesh<Scalar>::Face face;
            while (ss >> vert) {
                int vi = std::atoi(vert.substr(0, vert.find_first_of('/')).c_str());
                face.indices.push_back(vi - 1);
            }
            mesh.faces.push_back(face);
        }
    }

    Mesh<Scalar>::perFaceNormal(mesh);
    Mesh<Scalar>::perVertexNormal(mesh);
    file.close();
}

template<typename Scalar>
void Mesh<Scalar>::saveToObj(const std::string &filename, const Mesh<Scalar> &mesh, const Vector3<Scalar> &size) {
    std::ofstream ofs(filename);
    for (auto& vert : mesh.vertices) {
        ofs << "v " << vert.position.x() * size.x() << " " << vert.position.y() * size.y() << " " << vert.position.z() * size.z() << "\n";
        ofs << "vn " << vert.normal.x() << " " << vert.normal.y() << " " << vert.normal.z() << "\n";
    }
    for (auto& face: mesh.faces) {
        ofs << "f ";
        for (auto i : face.indices) {
            ofs << i + 1 << "//" << i + 1 << " ";
        }
        ofs << "\n";
    }
    ofs.close();
}
