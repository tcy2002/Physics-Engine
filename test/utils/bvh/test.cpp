#include <iostream>
#include <fstream>
#include <sstream>
#include "test_general.h"
#include "phys/shape/concave_mesh_shape.h"
#include "utils/bvh.h"
#include "intf/viewer.h"

void objToMesh(pe::Mesh& mesh, const std::string& filename) {
    std::fstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file " << filename << std::endl;
        return;
    }

    char buf[1024];
    while (file.getline(buf, 1024)) {
        std::stringstream ss(buf);
        std::string str;
        ss >> str;
        if (str == "v") {
            pe::Real x, y, z;
            ss >> x >> y >> z;
            mesh.vertices.push_back({ {x, y, z}, {0, 0, 0} });
        }
        else if (str == "f") {
            std::string vert;
            pe::Mesh::Face face;
            while (ss >> vert) {
                int vi = std::atoi(vert.substr(0, vert.find_first_of('/')).c_str());
                face.indices.push_back(vi - 1);
            }
            mesh.faces.push_back(face);
        }
    }

    pe::Mesh::perFaceNormal(mesh);
    for (auto& face : mesh.faces) {
        for (auto i : face.indices) {
            mesh.vertices[i].normal = face.normal;
        }
    }

    file.close();
}

void add_to_viewer(const pe::Mesh& mesh, const pe::Array<int> fs, pe::Array<int>& ids) {
    for (auto f : fs) {
        auto& face = mesh.faces[f];
        auto vert = mesh.vertices[face.indices[0]].position;
        int id = pe_intf::Viewer::addCube({ 0.02, 0.02, 0.02 });
        ids.push_back(id);
        pe::Transform trans(pe::Matrix3::identity(), vert);
        pe_intf::Viewer::updateTransform(id, pe_phys_shape::ShapeType::ST_Box, trans);
        pe_intf::Viewer::updateColor(id, pe_phys_shape::ShapeType::ST_Box, pe::Vector3(0.3, 0.8, 0.3));
    }
}

void testBVH() {
    pe::Mesh mesh;
    objToMesh(mesh, BVH_TEST_SOURCE_DIR "/bunny.obj");
    utils::BVH bvh(mesh, 68);

    pe_intf::Viewer::open("WorldTest", 800, 600,
        { 0, 10, 20 }, 0, PE_PI / pe::Real(12.0));
    pe_intf::Viewer::addMesh(mesh);
    pe::Array<int> ids;
    pe::Array<int> fs;
    bvh.search({0.5, 0.5, 0.5}, {2, 2, 2}, fs);
    add_to_viewer(mesh, fs, ids);
    std::cout << "node num: " << ids.size() << std::endl;
}

int main() {
    testBVH();
}