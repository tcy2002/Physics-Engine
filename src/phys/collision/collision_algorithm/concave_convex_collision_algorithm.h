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
		static bool testSepAxis(const pe::Mesh& meshA, const pe::Mesh::Face& faceA,
		                        const pe_phys_shape::Shape* object_b,
								const pe::Transform& transA, const pe::Transform& transB,
								const pe::Vector3& sep_axis, pe::Real& depth,
								pe::Vector3& witnessPointA, pe::Vector3& witnessPointB);
		static bool findSeparatingAxis(const pe::Vector3& normA,
			                           const pe_phys_shape::Shape* shapeB,
								       const pe::Mesh& meshA, const pe::Mesh::Face& faceA, const pe::Mesh& meshB,
								       const pe::Array<pe::Vector3>& uniqueEdgesA,
								       const pe::Array<pe::Vector3>& uniqueEdgesB,
						       		   const pe::Transform& transA, const pe::Transform& transB,
						       		   pe::Vector3& sep, pe::Real margin, ContactResult& result);
	};

} // pe_phys_collision