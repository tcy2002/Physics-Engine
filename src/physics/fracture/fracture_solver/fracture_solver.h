#pragma once

#include "physics/object/rigidbody.h"
#include "physics/object/fracturable_object.h"

namespace pe_physics_fracture {

enum FractureType {
    Sphere, Cylinder
};

struct FractureSource {
    FractureType type;
    pe::Vector3 position = pe::Vector3::zeros();
    pe::Vector3 intensity = pe::Vector3::ones();
};

class FractureSolver {
    COMMON_MEMBER_PTR_SET_GET(pe_physics_object::FracturableObject, fracturable_object, FracturableObject);

protected:
    pe::Array<pe_physics_object::RigidBody*> _result;

    static pe::Vector3 randomSpherePoints(pe::Real radius);
    static pe::Vector3 randomCylinderPoints(pe::Real radius, pe::Real height);
    static pe_physics_object::RigidBody* addMesh(pe::Mesh& mesh, const pe::Transform& trans, const std::string& obj_path);

    bool generatePoints(const pe::Array<FractureSource>& sources,
                        pe::Array<pe::Vector3>& points, pe::Array<pe::Vector3>& forces);

public:
    FractureSolver() = default;
    virtual ~FractureSolver() = default;

    virtual void solve(const pe::Array<FractureSource>& sources) = 0;
    pe::Array<pe_physics_object::RigidBody*>& getFragments() { return _result; }
    void clearFragments() { _result.clear(); }
};

} // namespace pe_physics_fracture
