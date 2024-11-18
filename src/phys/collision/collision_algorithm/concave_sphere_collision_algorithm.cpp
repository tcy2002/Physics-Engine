#include "concave_sphere_collision_algorithm.h"
#include "sphere_convex_collision_algorithm.h"
#include "phys/shape/sphere_shape.h"
#include "phys/shape/concave_mesh_shape.h"

namespace pe_phys_collision {

    bool ConcaveSphereCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                           pe::Transform trans_a, pe::Transform trans_b,
                                                           pe::Real refScale, ContactResult& result) {
        if (!((shape_a->getType() == pe_phys_shape::ShapeType::Sphere &&
            shape_b->getType() == pe_phys_shape::ShapeType::ConcaveMesh) ||
            (shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh &&
                shape_b->getType() == pe_phys_shape::ShapeType::Sphere))) {
            return false;
        }

        auto shape_mesh = (pe_phys_shape::ConcaveMeshShape*)(shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh ? shape_a : shape_b);
        auto& mesh = shape_mesh->getMesh();
        auto trans_mesh = shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh ? trans_a : trans_b;
        auto shape_sph = (pe_phys_shape::SphereShape*)(shape_a->getType() == pe_phys_shape::ShapeType::Sphere ? shape_a : shape_b);
        auto trans_sph = shape_a->getType() == pe_phys_shape::ShapeType::Sphere ? trans_a : trans_b;

        pe::Vector3 sph_rel2mesh = trans_mesh.inverseTransform(trans_sph.getOrigin());
        pe::Real radius = shape_sph->getRadius();
        pe::Vector3 sph_AA = sph_rel2mesh - pe::Vector3(radius, radius, radius);
        pe::Vector3 sph_BB = sph_rel2mesh + pe::Vector3(radius, radius, radius);
        pe::Array<int> intersect;
        shape_mesh->getIntersectFaces(sph_AA, sph_BB, intersect);

        pe::Vector3 vertices[3];
        result.setSwapFlag(shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh);
        for (auto fi : intersect) {
            auto& f = mesh.faces[fi];
            for (int i = 0; i < (int)f.indices.size() - 2; i++) {
                vertices[0] = mesh.vertices[f.indices[0]].position;
                vertices[1] = mesh.vertices[f.indices[i + 1]].position;
                vertices[2] = mesh.vertices[f.indices[i + 2]].position;
                SphereConvexCollisionAlgorithm::getClosestPoints(shape_sph, trans_sph, vertices, trans_mesh, result);
            }
        }
        result.setSwapFlag(false);

        return true;
    }

} // pe_phys_collision