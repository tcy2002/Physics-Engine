#pragma once

#include <vector>
#include <queue>
#include <unordered_map>

//// some common types
namespace pe {
    template <typename T>
    using Array = std::vector<T>;

    template <typename T>
    using Queue = std::queue<T>;

    template <typename K, typename V>
    using KVStore = std::unordered_map<K, V>;

    template <typename T1, typename T2>
    using Pair = std::pair<T1, T2>;
} // namespace pe

//// real type
#ifdef PE_USE_DOUBLE
typedef double PEReal;
#define PE_REAL_MAX __DBL_MAX__
#define PE_REAL_MIN __DBL_DENORM_MIN__
#else
typedef float PEReal;
#define PE_REAL_MAX __FLT_MAX__
#define PE_REAL_MIN __FLT_DENORM_MIN__
#endif

//// error tolerance
#define PE_EPS 1e-5
#define PE_APPROX_EQUAL(a, b) (std::abs((a) - (b)) < PE_EPS)

//// includes for different OS
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#elif defined LINUX
#include <sys/time.h>
#include <unistd.h>
#endif

//// inline: depends on OS
#ifdef _WIN32
#define PE_FORCE_INLINE __forceinline
#elif defined LINUX
#define PE_FORCE_INLINE __attribute__((always_inline))
#else
#define PE_FORCE_INLINE inline
#endif

//// timer: depends on OS
PE_FORCE_INLINE unsigned long long PE_GetTickCount() {
#ifdef _WIN32
    LARGE_INTEGER t, f;
    QueryPerformanceCounter(&t);
    QueryPerformanceFrequency(&f);
    return t.QuadPart * 1000 / f.QuadPart;
#elif defined LINUX
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
#else
    return 0;
#endif
}
#ifdef _WIN32
#define PE_Sleep(t) Sleep(t)
#elif defined LINUX
#define PE_Sleep(t) usleep(t * 1000)
#else
#define PE_Sleep(t)
#endif

//// member getter and setter
#define PE_BOOL_SET_GET(name, Name) \
protected: \
bool _##name; \
public: \
void set##Name(bool t){ _##name = t; } \
bool is##Name() const { return _##name; } \
private:
#define PE_BOOL_GET(name, Name) \
protected: \
bool _##name; \
public: \
bool is##Name() const { return _##name; } \
private:
#define PE_MEMBER_SET_GET(T, name, Name) \
protected: \
T _##name; \
public: \
void set##Name(const T& t){ _##name = t; } \
const T& get##Name() const { return _##name; } \
private:
#define PE_MEMBER_GET(T, name, Name) \
protected: \
T _##name; \
public: \
const T& get##Name() const { return _##name; } \
private:

// member pointer getter and setter
#define PE_MEMBER_PTR_SET_GET(T, name, Name) \
protected: \
T *_##name; \
public: \
void set##Name(T *t) { delete _##name; _##name = t; } \
T* get##Name() const { return _##name; } \
private:
#define PE_MEMBER_PTR_GET(T, name, Name) \
protected: \
T *_##name; \
public: \
T* get##Name() const { return _##name; } \
private:

// member getter and setter (atomic)
#include <atomic>
#define PE_BOOL_SET_GET_ATOMIC(name, Name) \
protected: \
std::atomic<bool> _##name; \
public: \
void set##Name(bool t){ _##name = t; } \
bool is##Name() const { return _##name; } \
private:
#define PE_BOOL_GET_ATOMIC(name, Name) \
protected: \
std::atomic<bool> _##name; \
public: \
bool is##Name() const { return _##name; } \
private:
