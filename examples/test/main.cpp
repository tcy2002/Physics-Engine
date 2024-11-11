#include <iostream>
#include <fstream>
#include <string>

#include "phys/shape/default_mesh.h"

int main() {
    pe::Mesh mesh = PE_BOX_DEFAULT_MESH;
    float vertices[72];
    for (int i = 0; i < 24; i++) {
        vertices[i * 3] = mesh.vertices[i].position.x;
        vertices[i * 3 + 1] = mesh.vertices[i].position.y;
        vertices[i * 3 + 2] = mesh.vertices[i].position.z;
    }
    float normals[72];
    for (int i = 0; i < 24; i++) {
        normals[i * 3] = mesh.vertices[i].normal.x;
        normals[i * 3 + 1] = mesh.vertices[i].normal.y;
        normals[i * 3 + 2] = mesh.vertices[i].normal.z;
    }
    unsigned int indices[36];
    for (int i = 0; i < 6; i++) {
        indices[i * 6] = mesh.faces[i].indices[0];
        indices[i * 6 + 1] = mesh.faces[i].indices[1];
        indices[i * 6 + 2] = mesh.faces[i].indices[2];
        indices[i * 6 + 3] = mesh.faces[i].indices[0];
        indices[i * 6 + 4] = mesh.faces[i].indices[2];
        indices[i * 6 + 5] = mesh.faces[i].indices[3];
    }
    float timesteps[3] = {0, 0.5, 1.0};
    float frames[9] = {0, 0, 0, 0.2, 0.2, 0.2, 0.4, 0.4, 0.4};
    float timesteps1[3] = {0.5, 1.0, 1.5};
    float frames1[9] = {0, 1.2, 0, 0.2, 1.4, 0.2, 0.4, 1.6, 0.4};
    std::ofstream file("cube.bin", std::ios::binary);
    file.write(reinterpret_cast<char*>(vertices), 72 * sizeof(float));
    file.write(reinterpret_cast<char*>(normals), 72 * sizeof(float));
    file.write(reinterpret_cast<char*>(indices), 36 * sizeof(unsigned int));
    file.write(reinterpret_cast<char*>(timesteps), 3 * sizeof(float));
    file.write(reinterpret_cast<char*>(frames), 9 * sizeof(float));
    file.write(reinterpret_cast<char*>(timesteps1), 3 * sizeof(float));
    file.write(reinterpret_cast<char*>(frames1), 9 * sizeof(float));
    file.close();
    mesh = PE_CYLINDER_DEFAULT_MESH;
    pe::Mesh::saveToObj("cube.obj", mesh, pe::Vector3::ones());
}