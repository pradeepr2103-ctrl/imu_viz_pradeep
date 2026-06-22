#pragma once
#include <cstring>
#include <cmath>

namespace mocap {

struct Vec3 { float x=0, y=0, z=0; };
struct Quat { float w=1, x=0, y=0, z=0; };

struct Mat4 {
    float m[16];  // column‑major

    Mat4() {
        std::memset(m, 0, sizeof(m));
        m[0] = m[5] = m[10] = m[15] = 1.0f;
    }

    static Mat4 perspective(float fovY, float aspect, float zn, float zf);
    static Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up);
};

inline Mat4 Mat4::perspective(float fovY, float aspect, float zn, float zf) {
    Mat4 r;
    std::memset(r.m, 0, sizeof(r.m));
    float f = 1.0f / std::tan(fovY * 0.5f);
    r.m[0] = f / aspect;
    r.m[5] = f;
    r.m[10] = (zf + zn) / (zn - zf);
    r.m[11] = -1.0f;
    r.m[14] = (2.0f * zf * zn) / (zn - zf);
    return r;
}

inline Mat4 Mat4::lookAt(const Vec3& eye, const Vec3& target, const Vec3& up) {
    Vec3 f = {target.x - eye.x, target.y - eye.y, target.z - eye.z};
    float fl = std::sqrt(f.x*f.x + f.y*f.y + f.z*f.z);
    f.x /= fl; f.y /= fl; f.z /= fl;

    Vec3 s = {
        f.y * up.z - f.z * up.y,
        f.z * up.x - f.x * up.z,
        f.x * up.y - f.y * up.x
    };
    Vec3 u = {
        s.y * f.z - s.z * f.y,
        s.z * f.x - s.x * f.z,
        s.x * f.y - s.y * f.x
    };

    Mat4 r;
    r.m[0] = s.x;  r.m[4] = s.y;  r.m[8]  = s.z;
    r.m[1] = u.x;  r.m[5] = u.y;  r.m[9]  = u.z;
    r.m[2] = -f.x; r.m[6] = -f.y; r.m[10] = -f.z;
    r.m[12] = -(s.x*eye.x + s.y*eye.y + s.z*eye.z);
    r.m[13] = -(u.x*eye.x + u.y*eye.y + u.z*eye.z);
    r.m[14] =  (f.x*eye.x + f.y*eye.y + f.z*eye.z);
    r.m[15] = 1.0f;
    return r;
}

} // namespace mocap