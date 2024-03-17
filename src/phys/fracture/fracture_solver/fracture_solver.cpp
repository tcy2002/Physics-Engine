#include "fracture_solver.h"
#include "phys/shape/convex_mesh_shape.h"
#include "phys/shape/box_shape.h"
#include <fstream>
#include <random>
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
            seed = COMMON_GetTickCount();
            std::cout << "fracture seed: " << seed << std::endl;
        }
        return seed;
    }

    pe::Vector3 FractureSolver::randomSpherePoints(pe::Real radius) {
        // edge contact and tiny fragment: 1013108201 1013255245
        // not stable: 1016601555
        // seems to pass through: 1019554970
        static std::default_random_engine e(236155093);
        static std::uniform_real_distribution<pe::Real> d(0., 1.);
        pe::Real theta = d(e) * 2 * PE_PI, alpha = (d(e) * 2 - 1) * PE_PI, roa = sqrt(d(e)) * radius;
        pe::Real cos_t = cos(theta), sin_t = sin(theta), cos_a = cos(alpha), sin_a = sin(alpha);
        return { roa * cos_t * cos_a, roa * sin_t * cos_a, roa * sin_a };
    }

    pe::Vector3 FractureSolver::randomCylinderPoints(pe::Real radius, pe::Real height) {
        static std::default_random_engine e(COMMON_GetTickCount());
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
        rb->setMass(calc_mesh_volume(mesh));
        rb->setLocalInertia(convexMesh->calcLocalInertia(rb->getMass()));
        rb->setFrictionCoeff(0.3);
        rb->setRestitutionCoeff(0.8);
        return rb;
    }

#   define EXPLOSION_RATE 0.125
#   define SPHERE_DENSITY 50
#   define CYLINDER_DENSITY 20

    void FractureSolver::solve(const pe::Array<FractureSource>& sources) {
        if (_fracturable_object == 0 || sources.empty()) return;

        // retrieve mesh data from different shapes
        pe_phys_shape::Shape* shape = _fracturable_object->getCollisionShape();
        pe::Mesh mesh;
        if (shape->getType() == pe_phys_shape::ShapeType::ConvexMesh) {
            mesh = ((pe_phys_shape::ConvexMeshShape*)(shape))->getMesh();
        } else if (shape->getType() == pe_phys_shape::ShapeType::Box) {
            mesh = pe_phys_shape::_box_mesh;
            auto size = ((pe_phys_shape::BoxShape*)(shape))->getSize();
            for (auto& vert : mesh.vertices) {
                vert.position.x *= size.x;
                vert.position.y *= size.y;
                vert.position.z *= size.z;
            }
        } else return;

        pe::Transform world_trans = _fracturable_object->getTransform();
        pe::Real threshold = _fracturable_object->getThreshold();

        // generate points
        pe::Array<pe::Vector3> points;
        for (auto& src : sources) {
            pe::Vector3 local_impact_pos = world_trans.inverseTransform(src.position);
            pe::Vector3 intensity = src.intensity;
            pe::Real impact_radius = (std::abs(intensity.x) + std::abs(intensity.y) + std::abs(intensity.z))
                                     / (3.0 * threshold);
            if (impact_radius < 0.01) continue;
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
            } else return;
        }
        if (points.size() <= 2) return;

        // calculate intensity for each point
        pe::Array<pe::Vector3> forces;
        forces.assign(points.size(), pe::Vector3::zeros());
        for (int i = 0; i < points.size(); i++) {
            pe::Vector3 pos = world_trans * points[i];
            for (auto& src : sources) {
                pe::Real expForce = src.intensity.norm();
                pe::Real dist = (pos - src.position).norm();
                pe::Vector3 dir = (pos - src.position).normalized();
                forces[i] += (dir * (expForce / dist * EXPLOSION_RATE));
            }
        }

        // generate new rigidbodies
        pe::Array<pe::Mesh> fragments;
        _calculator.triangulate(points);
        _calculator.fracture(mesh, fragments);
        for (int i = 0; i < fragments.size(); i++) {
            if (!fragments[i].empty()) {
                meshToObj(fragments[i], "test" + std::to_string(i) + ".obj");
                auto rb = addMesh(fragments[i], world_trans);
                pe::Vector3 vel = rb->getLinearVelocity();
                vel += forces[i] / rb->getMass();
                rb->setLinearVelocity(vel);
                _result.push_back(rb);
            }
        }
    }

} // namespace pe_phys_fracture