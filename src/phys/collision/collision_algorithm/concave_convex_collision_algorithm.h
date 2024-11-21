#pragma once

#include "phys/shape/concave_mesh_shape.h"
#include "phys/shape/convex_mesh_shape.h"
#include "collision_algorithm.h"

namespace pe_phys_collision {

	class ConcaveConvexCollisionAlgorithm : public CollisionAlgorithm {
	public:
		virtual bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                      pe::Transform trans_a, pe::Transform trans_b,
			                          pe::Real refScale, ContactResult& result) override;

		static bool getClosestPoints(pe_phys_shape::ConcaveMeshShape* shape_concave, pe_phys_shape::Shape* shape_convex,
		                             const pe::Transform& trans_concave, const pe::Transform& trans_convex,
		                             const pe::Array<pe::KV<pe::Vector3, pe::Vector3>>& unique_edges_convex,
		                             const pe::Mesh& mesh_concave, const pe::Mesh& mesh_convex,
		                             pe::Real margin, pe::Real refScale, ContactResult& result);
	};

} // pe_phys_collision