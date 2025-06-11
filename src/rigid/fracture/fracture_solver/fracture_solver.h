#pragma once

#include "rigid/object/rigidbody.h"
#include "rigid/object/fracturable_object.h"

namespace pe_phys_fracture {

    enum FractureType {
        Sphere, Cylinder
    };

    struct FractureSource {
        FractureType type;
        pe::Vector3 position = pe::Vector3::Zero();
        pe::Vector3 intensity = pe::Vector3::Ones();
    };

    class FractureSolver {
        COMMON_MEMBER_PTR_SET_GET(pe_phys_object::FracturableObject, fracturable_object, FracturableObject);

    protected:
        pe::Array<pe_phys_object::RigidBody*> _result;

        static pe::Vector3 randomSpherePoints(pe::Real radius);
        static pe::Vector3 randomCylinderPoints(pe::Real radius, pe::Real height);
        static pe_phys_object::RigidBody* addMesh(pe::Mesh& mesh, const pe::Transform& trans, const std::string& obj_path);

        bool generatePoints(const pe::Array<FractureSource>& sources,
                            pe::Array<pe::Vector3>& points, pe::Array<pe::Vector3>& forces);

    public:
        FractureSolver() {}
        virtual ~FractureSolver() {}

        virtual void solve(const pe::Array<FractureSource>& sources) = 0;
        pe::Array<pe_phys_object::RigidBody*>& getFragments() { return _result; }
        void clearFragments() { _result.clear(); }
    };

} // namespace pe_phys_fracture