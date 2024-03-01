#include "fracture_solver.h"
#include "phys/shape/convex_mesh_shape.h"
#include "phys/shape/box_shape.h"
#include <fstream>
#include <random>
#include "default_mesh.h"

namespace pe_phys_fracture {

    void FractureSolver::mesh_to_obj(const pe::Mesh &mesh, const std::string& obj_path) {
        std::ofstream ofs(obj_path);
        for (auto& vert : mesh.vertices) {
            ofs << "v " << vert.position.x << " " << vert.position.y << " " << vert.position.z << "\n";
        }
        auto size = mesh.faces.size();
        for (int i = 0; i < size; i++) {
            ofs << "f ";
            for (int j = 0; j < 3; j++) {
                ofs << mesh.faces[i].indices[j] + 1 << " ";
            }
            ofs << "\n";
        }
        ofs.close();
    }

    pe::Vector3 FractureSolver::random_sphere_points(pe::Real radius) {
        static std::default_random_engine e(COMMON_GetTickCount());
        static std::uniform_real_distribution<pe::Real> d(0., 1.);
        pe::Real theta = d(e) * 2 * PE_PI, alpha = (d(e) * 2 - 1) * PE_PI, roa = sqrt(d(e)) * radius;
        pe::Real cos_t = cos(theta), sin_t = sin(theta), cos_a = cos(alpha), sin_a = sin(alpha);
        return { roa * cos_t * cos_a, roa * sin_t * cos_a, roa * sin_a };
    }

    pe::Vector3 FractureSolver::random_cylinder_points(pe::Real radius, pe::Real height) {
        static std::default_random_engine e(COMMON_GetTickCount());
        static std::uniform_real_distribution<pe::Real> d(0., 1.);
        pe::Real theta = d(e) * 2 * PE_PI, roa = sqrt(d(e)) * radius, L = d(e), L2 = L * L * height;
        pe::Real cos_t = cos(theta), sin_t = sin(theta);
        return { roa * cos_t, roa * sin_t, L2 };
    }

    pe_phys_object::RigidBody* FractureSolver::add_mesh(const pe::Mesh& mesh, const pe::Transform& trans) {
        auto rb = new pe_phys_object::RigidBody();
        auto convexMesh = new pe_phys_shape::ConvexMeshShape(mesh);
        rb->setCollisionShape(convexMesh);
        rb->setTransform(trans);
        rb->setMass(calc_mesh_volume(mesh));
        rb->setFrictionCoeff(0.3);
        rb->setRestitutionCoeff(0);
        rb->setLinearVelocity(pe::Vector3::zeros());
        return rb;
    }

#define EXPLOSION_RATE 0.125
#define SPHERE_DENSITY 20
#define CYLINDER_DENSITY 10

    pe::Array<pe_phys_object::RigidBody*> FractureSolver::solve(pe_phys_object::FracturableObject* fb, const std::vector<FractureSource>& sources) {
        if (fb == 0 || sources.empty()) return {};

        // retrieve mesh data from different shapes
        pe_phys_shape::Shape* shape = fb->getCollisionShape();
        pe::Mesh mesh;
        if (shape->getType() == pe_phys_shape::ShapeType::ConvexMesh) {
            mesh = ((pe_phys_shape::ConvexMeshShape*)(shape))->getMesh();
        }
        else if (shape->getType() == pe_phys_shape::ShapeType::Box) {
            mesh = _cube_mesh;
            auto size = ((pe_phys_shape::BoxShape*)(shape))->getSize();
            for (auto& vert : mesh.vertices) {
                vert.position.x *= size.x;
                vert.position.y *= size.y;
                vert.position.z *= size.z;
            }
        }
        else return {};

        pe::Transform worldTrans = fb->getTransform();
        pe::Real threshold = fb->getThreshold();

        // generate points
        std::vector<pe::Vector3> points;
        for (auto& src : sources) {
            pe::Vector3 localImpactPos = worldTrans.inverseTransform(src.position);
            pe::Vector3 intensity = src.intensity;
            pe::Real impactRadius = (std::abs(intensity.x) + std::abs(intensity.y) + std::abs(intensity.z)) / 3 / threshold;
            if (impactRadius < 0.01) continue;
            if (src.type == FractureType::Sphere) {
                int pointCount = (int)(impactRadius * SPHERE_DENSITY);
                for (int i = 0; i < pointCount; i++) {
                    auto point = random_sphere_points(impactRadius) + localImpactPos;
                    if (shape->isInside(worldTrans, point)) {
                        points.push_back(point);
                    }
                }
            } else if (src.type == FractureType::Cylinder) {
                pe::Vector3 direction = (worldTrans.getBasis().transposed() * intensity).normalized();
                pe::Matrix3 rot = from_two_vectors(pe::Vector3(0, 0, 1), direction);
                int point_count = (int)(impactRadius * CYLINDER_DENSITY);
                for (int i = 0; i < point_count; i++) {
                    auto point = rot * random_cylinder_points(impactRadius / 5, impactRadius * 5) + localImpactPos;
                    if (shape->isInside(worldTrans, point)) points.push_back(point);
                }
            } else {
                return {};
            }
        }
        if (points.size() <= 2) return {};

        // calculate intensity for each point
        auto size = points.size();
        std::vector<pe::Vector3> forces;
        forces.assign(size, pe::Vector3::zeros());
        for (int i = 0; i < size; i++) {
            pe::Vector3 pos = worldTrans * points[i];
            for (auto& src : sources) {
                pe::Real expForce = src.intensity.norm();
                pe::Real dist = (pos - src.position).norm();
                pe::Vector3 dir = (pos - src.position).normalized();
                forces[i] += (dir * (expForce / dist * EXPLOSION_RATE));
            }
        }

        // generate new rigidbodies
        pe::Array<pe_phys_object::RigidBody*> new_rbs;
        pe::Array<pe::Mesh> fragments;
        _calculator.triangulate(points);
        _calculator.fracture(mesh, fragments);
        size = fragments.size();

        for (int i = 0; i < size; i++) {
            if (!fragments[i].empty()) {
                auto rb = add_mesh(fragments[i], worldTrans);
                pe::Vector3 vel = rb->getLinearVelocity();
                vel += forces[i] / rb->getMass();
                rb->setLinearVelocity(vel);
                new_rbs.push_back(rb);
            }
        }
        return new_rbs;
    }

} // namespace pe_phys_fracture