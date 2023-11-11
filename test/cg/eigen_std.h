#pragma once

#include "Eigen/Core"
#include "Eigen/Geometry"

#ifdef USE_DOUBLE
typedef Eigen::Vector3d Vector3Std;
typedef Eigen::Matrix3d Matrix3x3Std;
typedef Eigen::AngleAxisd AngleAxisStd;
#else
typedef Eigen::Vector3f Vector3Std;
typedef Eigen::Matrix3f Matrix3x3Std;
typedef Eigen::AngleAxisf AngleAxisStd;
#endif

#define ASSERT_VECTOR3_EQUAL(a, b) \
{ auto&& ao = a; auto&& bo = b; \
assert(EQUAL(ao.x, bo.x) && EQUAL(ao.y, bo.y) && EQUAL(ao.z, bo.z)); }

#define ASSERT_MATRIX3_EQUAL(a, b) \
{ auto&& ao = a; auto&& bo = b; \
assert(EQUAL(ao[0][0], bo[0][0]) && EQUAL(ao[0][1], bo[0][1]) && EQUAL(ao[0][2], bo[0][2]) && \
       EQUAL(ao[1][0], bo[1][0]) && EQUAL(ao[1][1], bo[1][1]) && EQUAL(ao[1][2], bo[1][2]) && \
       EQUAL(ao[2][0], bo[2][0]) && EQUAL(ao[2][1], bo[2][1]) && EQUAL(ao[2][2], bo[2][2])); }

#define ASSERT_TRANSFORM_EQUAL(t, b, o) \
{ auto&& to = t; auto&& basis = b; auto&& origin = o; \
ASSERT_MATRIX3_EQUAL(to.getBasis(), basis) \
ASSERT_VECTOR3_EQUAL(to.getOrigin(), origin) }

#define ASSERT_VECTOR3_EIGEN_EQUAL(a, b) \
{ auto&& ao = a; auto&& bo = b; \
assert(EQUAL(ao.x, bo.x()) && EQUAL(ao.y, bo.y()) && EQUAL(ao.z, bo.z()) && \
       EQUAL(ao[0], bo[0]) && EQUAL(ao[1], bo[1]) && EQUAL(ao[2], bo[2])); }

#define ASSERT_MATRIX3_EIGEN_EQUAL(a, b) \
{ auto&& ao = a; auto&& bo = b; \
assert(EQUAL(ao[0][0], bo(0, 0)) && EQUAL(ao[0][1], bo(0, 1)) && EQUAL(ao[0][2], bo(0, 2)) && \
       EQUAL(ao[1][0], bo(1, 0)) && EQUAL(ao[1][1], bo(1, 1)) && EQUAL(ao[1][2], bo(1, 2)) && \
       EQUAL(ao[2][0], bo(2, 0)) && EQUAL(ao[2][1], bo(2, 1)) && EQUAL(ao[2][2], bo(2, 2))); }

#define ASSERT_TRANSFORM_EIGEN_EQUAL(t, b, o) \
{ auto&& to = t; auto&& basis = b; auto&& origin = o; \
ASSERT_MATRIX3_EIGEN_EQUAL(to.getBasis(), basis) \
ASSERT_VECTOR3_EIGEN_EQUAL(to.getOrigin(), origin) }
