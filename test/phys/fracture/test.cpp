#include "phys/fracture/fracture_solver/simple_fracture_solver.h"
#include "phys/shape/box_shape.h"
#include "phys/shape/convex_mesh_shape.h"
#include <string>

using namespace pe_phys_fracture;

void testFracture() {
    FractureSource src;
    src.type = FractureType::Sphere;
    src.position = pe::Vector3(3.5, 3.5, 3.5);
    src.intensity = pe::Vector3(0.5, 0.5, 0.5);
//    src.type = FractureType::Cylinder;
//    src.position = pe::Vector3(2, 2, 0);
//    src.intensity = pe::Vector3(0, 0, 1.5);

    auto fo = new pe_phys_object::FracturableObject();
    fo->setThreshold(1.0);
    fo->setMass(1.0);
    fo->setCollisionShape(new pe_phys_shape::BoxShape(pe::Vector3(4, 4, 4)));
    fo->setTransform(pe::Transform(pe::Matrix3::Identity(), pe::Vector3(2, 2, 2)));

    FractureSolver* solver = new SimpleFractureSolver();
    solver->setFracturableObject(fo);
    solver->solve({src});
    int k = 0;
    for (auto frag : solver->getFragments()) {
        auto shape = (pe_phys_shape::ConvexMeshShape*)(frag->getCollisionShape());
        auto mesh = shape->getMesh();
        pe::Mesh::saveToObj("test" + std::to_string(k++) + ".obj", mesh, pe::Vector3::Ones());
    }
};

int main() {
    testFracture();
}