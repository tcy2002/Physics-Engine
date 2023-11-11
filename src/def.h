#pragma once

#include <vector>

namespace pe {
    template <typename T> using Array = std::vector<T>;
} // namespace pe

#ifdef USE_DOUBLE
typedef double real;
#define REAL_MAX __DBL_MAX__
#define REAL_MIN __DBL_DENORM_MIN__
#else
typedef float real;
#define REAL_MAX __FLT_MAX__
#define REAL_MIN __FLT_DENORM_MIN__
#endif

#ifdef _WIN32
#define PHYS_FORCE_INLINE __forceinline
#else
#define PHYS_FORCE_INLINE __attribute__ ((always_inline)) inline
#endif

#define MEMBER_SET_GET(T, name, Name) \
    protected: \
    T _##name; \
    public: \
    void set##Name(const T& t){ _##name = t; } \
    T const &get##Name() const { return _##name; } \
    private:

#define MEMBER_GET(T, name, Name) \
    protected: \
    T _##name; \
    public: \
    T const &get##Name() const { return _##name; } \
    private:
