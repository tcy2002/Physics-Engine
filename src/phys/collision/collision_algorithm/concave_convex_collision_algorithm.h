#pragma once

#include "collision_algorithm.h"

namespace pe_phys_collision {

	class ConcaveConvexCollisionAlgorithm : public CollisionAlgorithm {
	public:
		virtual bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                      pe::Transform trans_a, pe::Transform trans_b,
			                          ContactResult& result) override;
		static void getUniqueEdges(const pe::Mesh& mesh, const pe::Mesh::Face& face,
			                       pe::Array<pe::Vector3>& uniqueEdges);
		static bool testSepAxis(const pe_phys_shape::Shape* object_a,
		                        const pe::Mesh& meshB, const pe::Mesh::Face& faceB,
								const pe::Transform& trans_a, const pe::Transform& trans_b,
								const pe::Vector3& sep_axis, pe::Real& depth,
								pe::Vector3& witnessPointA, pe::Vector3& witnessPointB);
		static bool findSeparatingAxis(const pe_phys_shape::Shape* shapeA,
		                               const pe::Vector3& normB,
								       const pe::Mesh& meshA, const pe::Mesh& meshB, const pe::Mesh::Face& faceB,
								       const pe::Array<pe::Vector3>& uniqueEdgesA,
								       const pe::Array<pe::Vector3>& uniqueEdgesB,
						       		   const pe::Transform& transA, const pe::Transform& transB,
						       		   pe::Vector3& sep, pe::Real margin, ContactResult& result);
	};

} // pe_phys_collision