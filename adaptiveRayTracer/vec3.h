#ifndef VEC3_H
#define VEC3_H

#include <cmath>

struct Vec3 {
    double x, y, z;

    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(double t) const { return Vec3(x * t, y * t, z * t); }
    Vec3 operator/(double t) const { return Vec3(x / t, y / t, z / t); }

    double dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3 cross(const Vec3& v) const {
        return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }

    double length() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3 normalized() const {
        double l = length();
        if (l > 0) return Vec3(x / l, y / l, z / l);
        return Vec3(0, 0, 0);
    }

    Vec3 mult(const Vec3& v) const { return Vec3(x * v.x, y * v.y, z * v.z); }
};

#endif 