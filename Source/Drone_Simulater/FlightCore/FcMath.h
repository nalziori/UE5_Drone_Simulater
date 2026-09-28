// FlightCore — engine-independent quadrotor dynamics and control.
// Frame convention: right-handed, SI units. World: x forward, y left, z up.
// Body: x nose, y left wing, z up. Quaternions rotate body -> world.
#pragma once
#include <cmath>

namespace fc {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

inline double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(double ax, double ay, double az) : x(ax), y(ay), z(az) {}
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
    double norm() const { return std::sqrt(dot(*this)); }
    Vec3 normalized() const { double n = norm(); return n > 1e-12 ? *this / n : Vec3{}; }
    Vec3 cwise(const Vec3& o) const { return {x * o.x, y * o.y, z * o.z}; }
};

// Clamp vector length to maxLen (keeps direction).
inline Vec3 clampNorm(const Vec3& v, double maxLen) {
    double n = v.norm();
    return n > maxLen ? v * (maxLen / n) : v;
}

struct Quat {
    double w = 1, x = 0, y = 0, z = 0;
    Quat() = default;
    Quat(double aw, double ax, double ay, double az) : w(aw), x(ax), y(ay), z(az) {}

    Quat operator*(const Quat& o) const {
        return {w * o.w - x * o.x - y * o.y - z * o.z,
                w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x,
                w * o.z + x * o.y - y * o.x + z * o.w};
    }
    Quat conj() const { return {w, -x, -y, -z}; }
    Quat normalized() const {
        double n = std::sqrt(w * w + x * x + y * y + z * z);
        return {w / n, x / n, y / n, z / n};
    }
    Vec3 rotate(const Vec3& v) const {  // body -> world
        Quat r = (*this) * Quat{0, v.x, v.y, v.z} * conj();
        return {r.x, r.y, r.z};
    }
    Vec3 unrotate(const Vec3& v) const { return conj().rotate(v); }  // world -> body

    // ZYX (yaw-pitch-roll) Euler angles, radians.
    static Quat fromEuler(double roll, double pitch, double yaw) {
        double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
        double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
        double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
        return {cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy};
    }
    Vec3 toEuler() const {  // (roll, pitch, yaw)
        double roll = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
        double pitch = std::asin(clampd(2 * (w * y - z * x), -1.0, 1.0));
        double yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
        return {roll, pitch, yaw};
    }
    // Rotation matrix given as columns (body axes expressed in world).
    static Quat fromAxes(const Vec3& bx, const Vec3& by, const Vec3& bz) {
        double m00 = bx.x, m11 = by.y, m22 = bz.z, tr = m00 + m11 + m22;
        Quat q;
        if (tr > 0) {
            double s = std::sqrt(tr + 1.0) * 2;
            q = {0.25 * s, (by.z - bz.y) / s, (bz.x - bx.z) / s, (bx.y - by.x) / s};
        } else if (m00 > m11 && m00 > m22) {
            double s = std::sqrt(1.0 + m00 - m11 - m22) * 2;
            q = {(by.z - bz.y) / s, 0.25 * s, (by.x + bx.y) / s, (bz.x + bx.z) / s};
        } else if (m11 > m22) {
            double s = std::sqrt(1.0 + m11 - m00 - m22) * 2;
            q = {(bz.x - bx.z) / s, (by.x + bx.y) / s, 0.25 * s, (bz.y + by.z) / s};
        } else {
            double s = std::sqrt(1.0 + m22 - m00 - m11) * 2;
            q = {(bx.y - by.x) / s, (bz.x + bx.z) / s, (bz.y + by.z) / s, 0.25 * s};
        }
        return q.normalized();
    }
};

// Conversion to Unreal's left-handed frame (x forward, y RIGHT, z up), a mirror across y.
// Vectors flip y; rotations keep angle and mirror the axis as a pseudovector.
inline Vec3 toLeftHanded(const Vec3& v) { return {v.x, -v.y, v.z}; }
inline Quat toLeftHanded(const Quat& q) { return {q.w, -q.x, q.y, -q.z}; }
inline Vec3 fromLeftHanded(const Vec3& v) { return toLeftHanded(v); }  // mirror is its own inverse
inline Quat fromLeftHanded(const Quat& q) { return toLeftHanded(q); }

}  // namespace fc
