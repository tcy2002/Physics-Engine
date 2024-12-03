#pragma once

#include <iostream>
#include <random>
#include <sys/timeb.h>
#include "utils/logger.h"
#include "common/transform.h"

#define T_EPS 1e-4

// compare two real numbers
#define EQUAL(a, b) (std::abs((a) - (b)) < T_EPS)
// compare two integers
#define EQUAL_INT(a, b) ((a) == (b))
// compare two vector
#define ASSERT_VECTOR3_EQUAL(a, b) \
do { auto&& ao = a; auto&& bo = b; \
ASSERT(EQUAL(ao.x(), bo.x()) && EQUAL(ao.y(), bo.y()) && EQUAL(ao.z(), bo.z()), "vector3 not equal"); } while (0)
// compare two matrix
#define ASSERT_MATRIX3_EQUAL(a, b) \
do { auto&& ao = a; auto&& bo = b; \
ASSERT(EQUAL(ao(0, 0), bo(0, 0)) && EQUAL(ao(0, 1), bo(0, 1)) && EQUAL(ao(0, 2), bo(0, 2)) && \
       EQUAL(ao(1, 0), bo(1, 0)) && EQUAL(ao(1, 1), bo(1, 1)) && EQUAL(ao(1, 2), bo(1, 2)) && \
       EQUAL(ao(2, 0), bo(2, 0)) && EQUAL(ao(2, 1), bo(2, 1)) && EQUAL(ao(2, 2), bo(2, 2)), \
       "matrix3 not equal"); } while (0)

#define ASSERT(exp, msg) \
if (!(exp)) { \
    PE_LOG_ERROR << (msg) << std::endl; \
    exit(-1); \
}
#define ASSERT_EQUAL(a, b) ASSERT(EQUAL(a, b), "real not equal")
#define ASSERT_EQUAL_INT(a, b) ASSERT(EQUAL_INT(a, b), "int not equal")

#ifdef PE_USE_DOUBLE
typedef double Real;
#else
typedef float Real;
#endif

timeb t;

void randInit() {
    ftime(&t);
}

Real randR() {
    static std::default_random_engine e(t.millitm);
    static std::uniform_real_distribution<Real> u(-1e3, 1e3);
    return u(e);
}

Real randPos() {
    static std::default_random_engine e(t.millitm);
    static std::uniform_real_distribution<Real> u(1e-3, 1e3);
    return u(e);
}

Real randNeg() {
    static std::default_random_engine e(t.millitm);
    static std::uniform_real_distribution<Real> u(-1e3, -1e-3);
    return u(e);
}
