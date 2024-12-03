#pragma once

//// inline
#ifdef _WIN32
#define COMMON_FORCE_INLINE __forceinline
#else
#define COMMON_FORCE_INLINE inline
#endif

//// timer
#include <chrono>
COMMON_FORCE_INLINE unsigned long long COMMON_GetTickCount() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}
COMMON_FORCE_INLINE unsigned long long COMMON_GetMicroTickCount() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}
#define COMMON_USleep(us) std::this_thread::sleep_for(std::chrono::microseconds(us))

//// linear types
#include <Eigen/Core>
namespace common {
    template <typename Scalar> using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    template <typename Scalar> using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
    template <typename Scalar> using Quaternion = Eigen::Quaternion<Scalar>;
} // namespace common

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
void set##Name(T *t) { _##name = t; } \
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
