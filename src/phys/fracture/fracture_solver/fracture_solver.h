#pragma once

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
        COMMON_MEMBER_PTR_SET_GET(pe_phys_object::FracturableObject, fracturable_object, FracturableObject);

    protected:
        pe::Array<pe_phys_object::RigidBody*> _result;

        static pe::Vector3 randomSpherePoints(pe::Real radius);
        static pe::Vector3 randomCylinderPoints(pe::Real radius, pe::Real height);
        static pe_phys_object::RigidBody* addMesh(const pe::Mesh& mesh, const pe::Transform& trans);

    public:
        static void meshToObj(const pe::Mesh& mesh, const std::string& obj_path);

        virtual void solve(const pe::Array<FractureSource>& sources) = 0;
        pe::Array<pe_phys_object::RigidBody*>& getFragments() { return _result; }
    };

} // namespace pe_phys_fracture