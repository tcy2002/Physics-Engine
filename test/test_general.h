#pragma once

#include <iostream>
#include <random>
#include <sys/timeb.h>
#include <common/logger.h>

#define EPS 1e-5

// compare two real numbers
#define EQUAL(a, b) (std::abs((a) - (b)) < EPS)
// compare two integers
#define EQUAL_INT(a, b) ((a) == (b))

#define ASSERT(exp, msg) \
if (!(exp)) { \
    COMMON_LOG_ERROR << (msg) << std::endl; \
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
