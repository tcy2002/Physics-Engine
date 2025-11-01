#pragma once

#include "physics/shape/concave_mesh_shape.h"
#include "physics/shape/convex_mesh_shape.h"
#include "collision_algorithm.h"

namespace pe_physics_collision {

class ConcaveConvexCollisionAlgorithm : public CollisionAlgorithm {
public:
	bool processCollision(pe_physics_shape::Shape* shape_a, pe_physics_shape::Shape* shape_b,
	                      pe::Transform trans_a, pe::Transform trans_b,
	                      pe::Real refScale, ContactResult& result) override;

	static bool getClosestPoints(pe_physics_shape::ConcaveMeshShape* shape_concave, pe_physics_shape::Shape* shape_convex,
	                             const pe::Transform& trans_concave, const pe::Transform& trans_convex,
	                             const pe::Array<pe_physics_shape::UniqueEdge>& unique_edges_convex,
	                             const pe::Mesh& mesh_concave, const pe::Mesh& mesh_convex,
	                             pe::Real margin, pe::Real refScale, ContactResult& result);
};

} // pe_physics_collision
