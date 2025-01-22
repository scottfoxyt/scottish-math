#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>

using i8 = int8_t;
using i16 = int16_t;
using i32 = int32_t;
using i64 = int64_t;
using isize = ptrdiff_t;

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;
using usize = size_t;

using f32 = float;
using f64 = double;

struct i32vec3 {
    i32 x, y, z;

    bool operator ==(const i32vec3 &other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct u32vec3 {
    u32 x, y, z;

    bool operator ==(const u32vec3 &other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct f32vec3 {
    f32 x, y, z;

    f32vec3 normalize() {
        f32 mag = sqrt(x * x + y * y + z * z);
        return mag > 0.0 ? f32vec3{x / mag, y / mag, z / mag} : f32vec3{0.0};
    }

    f32vec3 operator +(const f32vec3 &rhs) const {
        return f32vec3{x + rhs.x, y + rhs.y, z + rhs.z};
    }

    f32vec3 &operator +=(const f32vec3 &rhs) {
        *this = *this + rhs;
        return *this;
    }

    f32vec3 operator -(const f32vec3 &rhs) const {
        return f32vec3{x - rhs.x, y - rhs.y, z - rhs.z};
    }

    f32vec3 &operator -=(const f32vec3 &rhs) {
        *this = *this - rhs;
        return *this;
    }

    bool operator ==(const f32vec3 &rhs) const {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }
};

struct f32quat {
    f32 x{0}, y{0}, z{0}, w{1};

    f32quat rotate(f32vec3 axis, f32 angle) {
        f32quat rot;
        rot.x = axis.x * sin(angle / 2);
        rot.y = axis.y * sin(angle / 2);
        rot.z = axis.z * sin(angle / 2);
        rot.w = cos(angle / 2);
        return rot * *this;
    }

    f32quat normalize() {
        f32 mag = sqrt(x * x + y * y + z * z + w * w);
        return mag > 0.0 ? f32quat{x / mag, y / mag, z / mag, w / mag} : f32quat{0.0};
    }

    operator f32mat4() const {
        f32mat4 mat;
        mat.m00 = 1 - 2 * (y * y + z * z);
        mat.m01 = 2 * (x * y - w * z);
        mat.m02 = 2 * (x * z + w * y);
        mat.m10 = 2 * (x * y + w * z);
        mat.m11 = 1 - 2 * (x * x + z * z);
        mat.m12 = 2 * (y * z - w * x);
        mat.m20 = 2 * (x * z - w * y);
        mat.m21 = 2 * (y * z + w * x);
        mat.m22 = 1 - 2 * (x * x + y * y);
        mat.m33 = 1;
        return mat;
    }

    f32quat operator *(const f32quat &rhs) const {
        return f32quat{
            w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
            w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
            w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w,
            w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z
        };
    }
};

struct f32mat4 {
    f32 m00, m01, m02, m03,
        m10, m11, m12, m13,
        m20, m21, m22, m23,
        m30, m31, m32, m33;

    f32mat4 operator *(const f32mat4 &rhs) const {
        f32mat4 result;
        result.m00 = m00 * rhs.m00 + m01 * rhs.m10 + m02 * rhs.m20 + m03 * rhs.m30;
        result.m01 = m00 * rhs.m01 + m01 * rhs.m11 + m02 * rhs.m21 + m03 * rhs.m31;
        result.m02 = m00 * rhs.m02 + m01 * rhs.m12 + m02 * rhs.m22 + m03 * rhs.m32;
        result.m03 = m00 * rhs.m03 + m01 * rhs.m13 + m02 * rhs.m23 + m03 * rhs.m33;
        result.m10 = m10 * rhs.m00 + m11 * rhs.m10 + m12 * rhs.m20 + m13 * rhs.m30;
        result.m11 = m10 * rhs.m01 + m11 * rhs.m11 + m12 * rhs.m21 + m13 * rhs.m31;
        result.m12 = m10 * rhs.m02 + m11 * rhs.m12 + m12 * rhs.m22 + m13 * rhs.m32;
        result.m13 = m10 * rhs.m03 + m11 * rhs.m13 + m12 * rhs.m23 + m13 * rhs.m33;
        result.m20 = m20 * rhs.m00 + m21 * rhs.m10 + m22 * rhs.m20 + m23 * rhs.m30;
        result.m21 = m20 * rhs.m01 + m21 * rhs.m11 + m22 * rhs.m21 + m23 * rhs.m31;
        result.m22 = m20 * rhs.m02 + m21 * rhs.m12 + m22 * rhs.m22 + m23 * rhs.m32;
        result.m23 = m20 * rhs.m03 + m21 * rhs.m13 + m22 * rhs.m23 + m23 * rhs.m33;
        result.m30 = m30 * rhs.m00 + m31 * rhs.m10 + m32 * rhs.m20 + m33 * rhs.m30;
        result.m31 = m30 * rhs.m01 + m31 * rhs.m11 + m32 * rhs.m21 + m33 * rhs.m31;
        result.m32 = m30 * rhs.m02 + m31 * rhs.m12 + m32 * rhs.m22 + m33 * rhs.m32;
        result.m33 = m30 * rhs.m03 + m31 * rhs.m13 + m32 * rhs.m23 + m33 * rhs.m33;
        return result;
    }
};

struct color {
    u8 r, g, b, a;

    bool operator ==(const color &rhs) const {
        return r == rhs.r && g == rhs.g && b == rhs.b && a == rhs.a;
    }
};
