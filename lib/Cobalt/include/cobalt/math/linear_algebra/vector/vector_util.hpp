#pragma once

#include <algorithm>
#include <sstream>
#include <iomanip>

#include "vector.hpp"

#include "../matrix/matrix.hpp"

namespace cobalt::math::linear_algebra {

// ---------------- Non-member Utility ----------------
/**
 *  @brief Clamp the elements of a vector between an interval
 *  @param v Vector to project.
 *  @param min Lower clamp bound.
 *  @param max Upper clamp bound.
 *  @return Element wise clamped vector v between [min, max]
 */
template<uint8_t N, typename T = float>
    constexpr inline Vector<N, T> clamp(const Vector<N, T> &v, float min, float max) {
        Vector<N, T> output;
        for(uint8_t i = 0; i < N; i++) {
            if(v[i] > max)      { output[i] = max; }
            else if(v[i] < min) { output[i] = min; }
            else                { output[i] = v[i]; }
        }

        return output;
    }

/**
 *  @brief Compute vector's element wise sign
 *  @param v Vector to sign check.
 *  @return Vector with sign of each element of v
 */
template<uint8_t N, typename T = float>
    constexpr inline Vector<N, T> sign(const Vector<N, T> &v) {
        Vector<N, T> output;
        for(uint8_t i = 0; i < N; i++) {
            if(v[i] > 0.0f)         { output[i] = static_cast<T>(1); }
            else if(v[i] < 0.0f)    { output[i] = static_cast<T>(-1); }
            else                    { output[i] = static_cast<T>(0); }
        }

        return output;
    }

/**
 *  @brief Compute element wise absolute value on vector
 *  @param v Vector to absolute value.
 *  @return Vector with absolute valued elements
 */
template<uint8_t N, typename T = float>
    constexpr inline Vector<N, T> abs(const Vector<N, T> &v) {
        Vector<N, T> output;
        for(uint8_t i = 0; i < N; i++) {
            output[i] = (v[i] >= 0.0f) ?v[i] :-v[i];
        }

        return output;
    }

/**
 *  @brief Compute smallest element of a vector
 *  @param v Vector to check.
 *  @return Smallest vector element
 */
template<uint8_t N, typename T = float>
    constexpr inline T min(const Vector<N, T> &v) {
        T output = v[0];

        for(uint8_t i = 0; i < N; i++) {
            if(v[i] < output ) { output = v[i]; }
        }

        return output;
    }

/**
 *  @brief Compute largest element of a vector
 *  @param v Vector to check.
 *  @return Largest vector element
 */
template<uint8_t N, typename T = float>
    constexpr inline T max(const Vector<N, T> &v) {
        T output = v[0];

        for(uint8_t i = 0; i < N; i++) {
            if(v[i] > output ) { output = v[i]; }
        }

        return output;
    }

/**
 *  @brief Compute projection of a 3D-vector on to a plane (plane normal).
 *  @param v Vector to project.
 *  @param n Unit plane normal.
 *  @return Vector v projected onto the plane of n.
 */
template<uint8_t N, typename T = float>
    constexpr inline Vector<N, T> projectOntoPlane(const Vector<N, T> &v, const Vector<N, T> &n) {
        return v - dot(v, n)*n;
    }

/**
 *  @brief Compute component of one vector orthogonal to another.
 *  @param v Vector to reject.
 *  @param u Vector to reject from.
 *  @return Orthogonal component of v to u.
 */
template<uint8_t N, typename T = float>
    constexpr inline Vector<N, T> rejectFrom(const Vector<N, T> &v, const Vector<N, T> &u) {
        return v - projectOnto(v, u);
    }

/**
 *  @brief Compute reflection of a vector across a plane normal.
 *  @param v Vector to reflect.
 *  @param n Unit plane normal.
 *  @return Reflected vector v relative to the plane of n
 */
template<uint8_t N, typename T = float>
    constexpr inline Vector<N, T> reflect(const Vector<N, T> &v, const Vector<N, T> &n) {
        return v - 2.0f*dot(v, n)*n;
    }

/**
 *  @brief Compute sum of all elements of a vector.
 *  @param v Vector to sum across.
 */
template<uint8_t N, typename T = float>
    constexpr inline T sumElements(const Vector<N, T> &v) {
        T output = static_cast<T>(0);
        for(uint8_t i = 0; i < N; i++) {
            output += v[i];
        } 
        return output;
    }

/**
 *  @brief Compute product of all elements of a vector.
 *  @param v Vector to multiply across.
 */
template<uint8_t N, typename T = float>
    constexpr inline T productElements(const Vector<N, T> &v) {
        T output = static_cast<T>(1);
        for(uint8_t i = 0; i < N; i++) {
            output *= v[i];
        } 
        return output;
    }


/**
 *  @brief Linear interpolation between two vectors.
 *  @param v Start vector.
 *  @param u End vector.
 *  @param t Interpolation factor (0.0 → v, 1.0 → u).
 *  @return Interpolated vector between v and u.
 */
template<uint8_t N, typename T = float>
constexpr inline Vector<N, T> lerp(const Vector<N, T> &v,const Vector<N, T> &u, float t) {
    return v*(1.0f - t) + u*t;
}

/**
 *  @brief Spherical interpolation between two vectors.
 *  @param v Start vector.
 *  @param u End vector.
 *  @param t Interpolation factor (0.0 → v, 1.0 → u)
 *  @return Spherically interpolated vector between v and u.
 */
template<uint8_t N, typename T = float>
constexpr inline Vector<N, T> slerp(Vector<N, T> v, Vector<N, T> u, float t) {
    v = normalize(v);
    u = normalize(u);

    float dotVU = dot(v, u);
    dotVU = std::clamp(dotVU, -1.0f, 1.0f);

    float theta = acosf(dotVU) * t;

    Vector<N, T> relative = normalize(u - v*dotVU);

    return v*cosf(theta) + relative*sinf(theta);
}

// ---------------- Conversions ----------------
/**
 *  @brief Convert a vector to std::array.
 */
template<uint8_t N, typename T = float>
constexpr inline std::array<T, N> toArray(const Vector<N, T> &v) {
    std::array<T, N> arr{};
    for(uint8_t i = 0; i < N; i++) {
        arr[i] = v[i];
    }

    return arr;
}

/**
 *  @brief Construct a skew-symmetric matrix(3x3)from a vector(3).
 *  @param v Vector to turn into a skew-symmetric matrix.
 */
template<typename T = float>
constexpr inline Matrix<3, 3, T> skew(const Vector<3, T> &v) {
    Matrix<3, 3, T> output = Matrix<3, 3, T>::zero();

    output(0, 1) = -v.z();
    output(0, 2) =  v.y();
    output(1, 2) = -v.x();

    output(1, 0) =  v.z();
    output(2, 0) = -v.y();
    output(2, 1) =  v.x();

    return output;
}

// ---------------- Checks ----------------
/**
 *  @brief Check if a vector is normalized (norm = 1)
 */
template<uint8_t N, typename T = float>
constexpr inline bool isNormalized(const Vector<N, T> &v) {
    return (fabsf(norm(v) - 1.0f) < VECTOR_EQUAL_THRESHOLD);
}

} // cobalt::math::linear_algebra