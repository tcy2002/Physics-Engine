#pragma once

//// includes
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#else
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#endif

//// inline
#ifdef _WIN32
#define COMMON_FORCE_INLINE __forceinline
#else
#define COMMON_FORCE_INLINE inline
#endif

//// timer
COMMON_FORCE_INLINE unsigned long long COMMON_GetTickCount() {
#ifdef _WIN32
    LARGE_INTEGER t, f;
    QueryPerformanceCounter(&t);
    QueryPerformanceFrequency(&f);
    return t.QuadPart * 1000 / f.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (unsigned long long)(ts.tv_nsec / 1000000) + 
           ((unsigned long long)ts.tv_sec * 1000ull);
#endif
}
#ifdef _WIN32
#define COMMON_Sleep(t) Sleep((t) > 0 ? (t) : 0)
#else
#define COMMON_Sleep(t) usleep((t) > 0 ? (t) * 1000 : 0)
#endif

//// member getter and setter
#define COMMON_BOOL_SET_GET(name, Name) \
protected: \
bool _##name; \
public: \
void set##Name(bool t){ _##name = t; } \
bool is##Name() const { return _##name; } \
private:
#define COMMON_BOOL_GET(name, Name) \
protected: \
bool _##name; \
public: \
bool is##Name() const { return _##name; } \
private:
#define COMMON_MEMBER_SET_GET(T, name, Name) \
protected: \
T _##name; \
public: \
void set##Name(const T& t){ _##name = t; } \
const T& get##Name() const { return _##name; } \
private:
#define COMMON_MEMBER_GET(T, name, Name) \
protected: \
T _##name; \
public: \
const T& get##Name() const { return _##name; } \
private:

// member pointer getter and setter
#define COMMON_MEMBER_PTR_SET_GET(T, name, Name) \
protected: \
T* _##name = nullptr; \
public: \
void set##Name(T *t) { delete _##name; _##name = t; } \
T* get##Name() const { return _##name; } \
private:
#define COMMON_MEMBER_PTR_GET(T, name, Name) \
protected: \
T* _##name = nullptr; \
public: \
T* get##Name() const { return _##name; } \
private:

// member getter and setter (atomic)
#include <atomic>
#define COMMON_BOOL_SET_GET_ATOMIC(name, Name) \
protected: \
std::atomic<bool> _##name; \
public: \
void set##Name(bool t){ _##name.store(t); } \
bool is##Name() const { return _##name.load(); } \
private:
#define COMMON_BOOL_GET_ATOMIC(name, Name) \
protected: \
std::atomic<bool> _##name; \
public: \
bool is##Name() const { return _##name.load(); } \
private:
#define COMMON_MEMBER_SET_GET_ATOMIC(T, name, Name) \
protected: \
std::atomic<T> _##name; \
public: \
void set##Name(const T& t){ _##name.store(t); } \
T get##Name() const { return _##name.load(); } \
private:
#define COMMON_MEMBER_GET_ATOMIC(T, name, Name) \
protected: \
std::atomic<T> _##name; \
public: \
T get##Name() const { return _##name.load(); } \
private:
