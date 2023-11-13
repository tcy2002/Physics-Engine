#pragma once

#include <cassert>
#include <random>
#include <sys/timeb.h>

#include "../src/def.h"

#define EPS 1e-5

#define EQUAL(a, b) (std::abs((a) - (b)) < EPS)
#define EQUAL_INT(a, b) ((a) == (b))

#define ASSERT_EQUAL(a, b) assert(EQUAL(a, b));
#define ASSERT_EQUAL_INT(a, b) assert(EQUAL_INT(a, b));

timeb t;

void randInit() {
    ftime(&t);
}

real randR() {
    static std::default_random_engine e(t.millitm);
    static std::uniform_real_distribution<real> u(-1e3, 1e3);
    return u(e);
}

real randPos() {
    static std::default_random_engine e(t.millitm);
    static std::uniform_real_distribution<real> u(1e-3, 1e3);
    return u(e);
}

real randNeg() {
    static std::default_random_engine e(t.millitm);
    static std::uniform_real_distribution<real> u(-1e3, -1e-3);
    return u(e);
}
