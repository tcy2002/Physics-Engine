#include "fracture_solver.h"
#include "phys/shape/convex_mesh_shape.h"
#include "phys/shape/box_shape.h"
#include <fstream>
#include <random>
#include <string>
#include "phys/shape/default_mesh.h"

namespace pe_phys_fracture {

    void FractureSolver::meshToObj(const pe::Mesh &mesh, const std::string& obj_path) {
        std::ofstream ofs(obj_path);
        for (auto& vert : mesh.vertices) {
            ofs << "v " << vert.position.x << " " << vert.position.y << " " << vert.position.z << "\n";
        }
        for (auto& face: mesh.faces) {
            ofs << "f ";
            for (auto i : face.indices) {
                ofs << i + 1 << " ";
            }
            ofs << "\n";
        }
        ofs.close();
    }

    uint32_t generateSeed() {
        static uint32_t seed = -1;
        if (seed == -1) {
            seed = (uint32_t)COMMON_GetTickCount();
            std::cout << "fracture seed: " << seed << std::endl;
        }
        return seed;
    }

    pe::Vector3 FractureSolver::randomSpherePoints(pe::Real radius) {
        static std::default_random_engine e(999);
        static std::uniform_real_distribution<pe::Real> d(0., 1.);
        pe::Real theta = d(e) * 2 * PE_PI, alpha = (d(e) * 2 - 1) * PE_PI, roa = sqrt(d(e)) * radius;
        pe::Real cos_t = cos(theta), sin_t = sin(theta), cos_a = cos(alpha), sin_a = sin(alpha);
        return { roa * cos_t * cos_a, roa * sin_t * cos_a, roa * sin_a };
    }

    pe::Vector3 FractureSolver::randomCylinderPoints(pe::Real radius, pe::Real height) {
        static std::default_random_engine e((uint32_t)COMMON_GetTickCount());
        static std::uniform_real_distribution<pe::Real> d(0., 1.);
        pe::Real theta = d(e) * 2 * PE_PI, roa = sqrt(d(e)) * radius, L = d(e), L2 = L * L * height;
        pe::Real cos_t = cos(theta), sin_t = sin(theta);
        return { roa * cos_t, roa * sin_t, L2 };
    }

    pe_phys_object::RigidBody* FractureSolver::addMesh(const pe::Mesh& mesh, const pe::Transform& trans) {
        auto rb = new pe_phys_object::RigidBody();
        auto convexMesh = new pe_phys_shape::ConvexMeshShape();
        pe::Vector3 offset = convexMesh->setMesh(mesh);
        rb->setCollisionShape(convexMesh);
        rb->setTransform(pe::Transform(trans.getBasis(), trans.getOrigin() + offset));
        rb->setMass(pe_phys_shape::ConvexMeshShape::calcMeshVolume(mesh));
        rb->setLocalInertia(convexMesh->calcLocalInertia(rb->getMass()));
        rb->setFrictionCoeff(0.3);
        rb->setRestitutionCoeff(0.8);
        return rb;
    }

} // namespace pe_phys_fracture