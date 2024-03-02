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
        pe::Vector3 intensity;
    };

    class FractureSolver {
    private:
        FractureCalculator _calculator{};

        COMMON_MEMBER_PTR_SET_GET(pe_phys_object::FracturableObject, fracturable_object, FracturableObject);
        pe::Array<pe_phys_object::RigidBody*> _result;

        static pe::Vector3 randomSpherePoints(pe::Real radius);
        static pe::Vector3 randomCylinderPoints(pe::Real radius, pe::Real height);
        static pe_phys_object::RigidBody* addMesh(const pe::Mesh& mesh, const pe::Transform& trans);

    public:
        static void meshToObj(const pe::Mesh& mesh, const std::string& obj_path);
        void solve(const pe::Array<FractureSource>& sources);
        pe::Array<pe_phys_object::RigidBody*>& getFragments() { return _result; }
    };

} // namespace pe_phys_fracture