#pragma once

#include "common/vector3.h"
#include "common/matrix3x3.h"
#include "common/transform.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "test_def.h"

#ifdef PE_USE_DOUBLE
typedef Eigen::Vector3d Vector3Std;
typedef Eigen::Matrix3d Matrix3x3Std;
typedef Eigen::AngleAxisd AngleAxisStd;
typedef common::Vector3<double> Vector3Test;
typedef common::Matrix3x3<double> Matrix3Test;
typedef common::Transform<double> TransformTest;
#else
typedef Eigen::Vector3f Vector3Std;
typedef Eigen::Matrix3f Matrix3x3Std;
typedef Eigen::AngleAxisf AngleAxisStd;
typedef common::Vector3<float> Vector3Test;
typedef common::Matrix3x3<float> Matrix3Test;
typedef common::Transform<float> TransformTest;
#endif

#define ASSERT_VECTOR3_EQUAL(a, b) \
{ auto&& ao = a; auto&& bo = b; \
ASSERT(EQUAL(ao.x, bo.x) && EQUAL(ao.y, bo.y) && EQUAL(ao.z, bo.z), "vector3 not equal"); }

#define ASSERT_MATRIX3_EQUAL(a, b) \
{ auto&& ao = a; auto&& bo = b; \
ASSERT(EQUAL(ao[0][0], bo[0][0]) && EQUAL(ao[0][1], bo[0][1]) && EQUAL(ao[0][2], bo[0][2]) && \
       EQUAL(ao[1][0], bo[1][0]) && EQUAL(ao[1][1], bo[1][1]) && EQUAL(ao[1][2], bo[1][2]) && \
       EQUAL(ao[2][0], bo[2][0]) && EQUAL(ao[2][1], bo[2][1]) && EQUAL(ao[2][2], bo[2][2]), \
       "matrix3 not equal"); }

#define ASSERT_TRANSFORM_EQUAL(t, b, o) \
{ auto&& to = t; auto&& basis = b; auto&& origin = o; \
ASSERT_MATRIX3_EQUAL(to.getBasis(), basis) \
ASSERT_VECTOR3_EQUAL(to.getOrigin(), origin) }

#define ASSERT_VECTOR3_EIGEN_EQUAL(a, b) \
{ auto&& ao = a; auto&& bo = b; \
ASSERT(EQUAL(ao.x, bo.x()) && EQUAL(ao.y, bo.y()) && EQUAL(ao.z, bo.z()) && \
       EQUAL(ao[0], bo[0]) && EQUAL(ao[1], bo[1]) && EQUAL(ao[2], bo[2]), \
       "vector3 not equal to eigen"); }

#define ASSERT_MATRIX3_EIGEN_EQUAL(a, b) \
{ auto&& ao = a; auto&& bo = b; \
assert(EQUAL(ao[0][0], bo(0, 0)) && EQUAL(ao[0][1], bo(0, 1)) && EQUAL(ao[0][2], bo(0, 2)) && \
       EQUAL(ao[1][0], bo(1, 0)) && EQUAL(ao[1][1], bo(1, 1)) && EQUAL(ao[1][2], bo(1, 2)) && \
       EQUAL(ao[2][0], bo(2, 0)) && EQUAL(ao[2][1], bo(2, 1)) && EQUAL(ao[2][2], bo(2, 2)) \
       "matrix3 not equal to eigen"); }

#define ASSERT_TRANSFORM_EIGEN_EQUAL(t, b, o) \
{ auto&& to = t; auto&& basis = b; auto&& origin = o; \
ASSERT_MATRIX3_EIGEN_EQUAL(to.getBasis(), basis) \
ASSERT_VECTOR3_EIGEN_EQUAL(to.getOrigin(), origin) }
