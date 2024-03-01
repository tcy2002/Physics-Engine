#pragma once

#include "fracture_calculator.h"
#include "phys/object/rigidbody.h"
#include "phys/object/fracturable_object.h"

namespace pe_phys_fracture {

    enum FractureType {
        Sphere, Cylinder
    };

    struct FractureSource {
        FractureType type;
        pe::Vector3 position;
        pe::Vector3 normal;
        pe::Vector3 intensity;
    };

    class FractureSolver {
    private:
        FractureCalculator _calculator{};

        static void mesh_to_obj(const pe::Mesh& mesh, const std::string& obj_path);
        static pe::Vector3 random_sphere_points(pe::Real radius);
        static pe::Vector3 random_cylinder_points(pe::Real radius, pe::Real height);
        static pe_phys_object::RigidBody* add_mesh(const pe::Mesh& mesh, const pe::Transform& trans);

    public:
        pe::Array<pe_phys_object::RigidBody*> solve(pe_phys_object::FracturableObject* fo, const std::vector<FractureSource>& sources);
    };

} // namespace pe_phys_fracture