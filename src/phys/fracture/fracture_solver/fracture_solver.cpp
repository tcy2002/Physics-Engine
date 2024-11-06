#include "fracture_solver.h"
#include "phys/shape/convex_mesh_shape.h"
#include "phys/shape/box_shape.h"
#include <fstream>
#include <random>
#include <string>
#include "phys/shape/default_mesh.h"
#include "phys/fracture/fracture_utils/fracture_utils.h"

namespace pe_phys_fracture {

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

#   define EXPLOSION_RATE pe::Real(0.125)
#   define SPHERE_DENSITY 50
#   define CYLINDER_DENSITY 20

    bool FractureSolver::generatePoints(const pe::Array<FractureSource> &sources,
                                        pe::Array<pe::Vector3> &points, pe::Array<pe::Vector3>& forces) {
        pe_phys_shape::Shape* shape = _fracturable_object->getCollisionShape();
        pe::Transform world_trans = _fracturable_object->getTransform();
        pe::Real threshold = _fracturable_object->getThreshold();

        for (auto& src : sources) {
            pe::Vector3 local_impact_pos = world_trans.inverseTransform(src.position);
            pe::Vector3 intensity = src.intensity;
            pe::Real impact_radius = (PE_ABS(intensity.x) + PE_ABS(intensity.y) + PE_ABS(intensity.z))
                                     / (pe::Real(3.0) * threshold);
            if (impact_radius < pe::Real(0.01)) continue;
            if (src.type == FractureType::Sphere) {
                int point_count = (int)(impact_radius * SPHERE_DENSITY);
                for (int i = 0; i < point_count; i++) {
                    auto point = randomSpherePoints(impact_radius) + local_impact_pos;
                    if (shape->localIsInside(point)) {
                        points.push_back(point);
                    }
                }
            } else if (src.type == FractureType::Cylinder) {
                pe::Vector3 direction = (world_trans.getBasis().transposed() * intensity).normalized();
                pe::Matrix3 rot = from_two_vectors(pe::Vector3(0, 0, 1), direction);
                int point_count = (int)(impact_radius * CYLINDER_DENSITY);
                for (int i = 0; i < point_count; i++) {
                    auto point = rot * randomCylinderPoints(impact_radius / 5,
                                                            impact_radius * 5) + local_impact_pos;
                    if (shape->localIsInside(point)) {
                        points.push_back(point);
                    }
                }
            } else return false;
        }
        if (points.size() <= 2) return false;

        forces.assign(points.size(), pe::Vector3::zeros());
        for (int i = 0; i < (int)points.size(); i++) {
            pe::Vector3 pos = world_trans * points[i];
            for (auto& src : sources) {
                pe::Real expForce = src.intensity.norm();
                pe::Real dist = (pos - src.position).norm();
                pe::Vector3 dir = (pos - src.position).normalized();
                forces[i] += (dir * (expForce / dist * EXPLOSION_RATE));
            }
        }

        return true;
    }

    pe_phys_object::RigidBody* FractureSolver::addMesh(pe::Mesh& mesh, const pe::Transform& trans, const std::string& obj_path) {
        auto rb = new pe_phys_object::RigidBody();
        auto convexMesh = new pe_phys_shape::ConvexMeshShape();
        convexMesh->setMeshPath(obj_path);
        convexMesh->setScale(pe::Vector3::ones());
        pe::Vector3 offset = convexMesh->setMesh(mesh);
        rb->setCollisionShape(convexMesh);
        rb->setTransform(pe::Transform(trans.getBasis(), trans.getOrigin() + offset));
        rb->setMass(convexMesh->getVolume());
        rb->setFrictionCoeff(pe::Real(0.3));
        rb->setRestitutionCoeff(pe::Real(0.8));
        return rb;
    }

} // namespace pe_phys_fracture