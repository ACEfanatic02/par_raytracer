#pragma once
#include <cmath>

#define Assert(...)

///////////////////////////////////////////////////////////////////////////////

#ifdef min
#undef min
#endif
#define min(a, b) ((a) < (b) ? (a) : (b))

#ifdef max
#undef max
#endif
#define max(a, b) ((a) > (b) ? (a) : (b))

#define Clamp(n, a, b) (min(max(n, a), b))
#define Lerp(a, b, t) ((a) + ((b) - (a))*(t))

inline float 
Fract(float f) {
    return f - (s32)f;
}

///////////////////////////////////////////////////////////////////////////////

struct Vector2 {
    float x;
    float y;
    float z;

    Vector2() {
        Set(0.0f, 0.0f);
    }

    Vector2(float x, float y) {
        Set(x, y);
    }

    Vector2(const Vector2& v) {
        Set(v.x, v.y);
    }

#define VEC_MUTOP_V(op) \
    inline Vector2& \
    operator op (Vector2 v) {\
        Set(x op v.x, y op v.y);\
        return *this;\
    }

    VEC_MUTOP_V(+=)
    VEC_MUTOP_V(-=)
    VEC_MUTOP_V(*=)
    VEC_MUTOP_V(/=)

#undef VEC_MUTOP_V

    inline void
    Set(float x, float y) {
        this->x = x;
        this->y = y;
    }
};

inline bool
operator==(Vector2 a, Vector2 b) {
    return a.x == b.x && a.y == b.y;
}

inline bool
operator!=(Vector2 a, Vector2 b) {
    return !(a == b);
}

#define VEC_BINOP_V(op) \
inline Vector2 \
operator op (Vector2 a, Vector2 b) {\
    return Vector2(a.x op b.x, a.y op b.y);\
}

VEC_BINOP_V(+);
VEC_BINOP_V(-);
VEC_BINOP_V(*);
VEC_BINOP_V(/);
#undef VEC_BINOP_V

inline Vector2
operator*(Vector2 a, float b) {
    return Vector2(a.x * b, a.y * b);
}

inline Vector2
operator/(Vector2 a, float b) {
    return Vector2(a.x / b, a.y / b);
}

inline float
Dot(Vector2 a, Vector2 b) {
    return a.x * b.x + a.y * b.y;
}

inline Vector2
Perp(Vector2 a) {
    return Vector2(-a.y, a.x);
}

inline float
Length(Vector2 a) {
    return sqrtf(Dot(a, a));
}

inline Vector2 
Normalize(Vector2 a) {
    float length_sq = Dot(a, a);
    if (length_sq == 0.0f) {
        return a;
    }
    else {
        return a / sqrtf(length_sq);
    }
}

///////////////////////////////////////////////////////////////////////////////

struct Vector3 {
    float x;
    float y;
    float z;

    Vector3() {
        Set(0.0f, 0.0f, 0.0f);
    }

    Vector3(float x, float y, float z) {
        Set(x, y, z);
    }

    Vector3(Vector3& v) {
        Set(v.x, v.y, v.z);
    }

#define VEC_MUTOP_V(op) \
    inline Vector3& \
    operator op (Vector3 v) {\
        Set(x op v.x, y op v.y, z op v.z);\
        return *this;\
    }

    VEC_MUTOP_V(+=)
    VEC_MUTOP_V(-=)
    VEC_MUTOP_V(*=)
    VEC_MUTOP_V(/=)

#undef VEC_MUTOP_V

    inline Vector3& 
    operator*=(float scale) {
        Set(x * scale, y * scale, z * scale);
        return *this;
    }

    inline Vector3& 
    operator/=(float scale) {
        Set(x / scale, y / scale, z / scale);
        return *this;
    }

    inline void
    Set(float x, float y, float z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }
};

inline bool
operator==(Vector3 a, Vector3 b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

inline bool
operator!=(Vector3 a, Vector3 b) {
    return !(a == b);
}

#define VEC_BINOP_V(op) \
inline Vector3 \
operator op (Vector3 a, Vector3 b) {\
    return Vector3(a.x op b.x, a.y op b.y, a.z op b.z);\
}

VEC_BINOP_V(+);
VEC_BINOP_V(-);
VEC_BINOP_V(*);
VEC_BINOP_V(/);
#undef VEC_BINOP_V

inline Vector3
operator*(Vector3 a, float b) {
    return Vector3(a.x * b, a.y * b, a.z * b);
}

inline Vector3
operator/(Vector3 a, float b) {
    return Vector3(a.x / b, a.y / b, a.z / b);
}

inline float
Dot(Vector3 a, Vector3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vector3
Cross(Vector3 a, Vector3 b) {
    return Vector3(
            a.y*b.z - b.y*a.z,
            a.z*b.x - b.z*a.x,
            a.x*b.y - b.x*a.y
        );
}

inline float
Length(Vector3 v) {
    return sqrtf(Dot(v, v));
}

inline Vector3
Normalize(Vector3 a) {
    float length_sq = Dot(a, a);
    if (length_sq == 0.0f) {
        return a;
    }
    else {
        return a / sqrtf(length_sq);
    }
}

///////////////////////////////////////////////////////////////////////////////

struct Vector4 {
    float x;
    float y;
    float z;
    float w;

    Vector4() {
        Set(0.0f, 0.0f, 0.0f, 0.0f);
    }

    Vector4(float x, float y, float z, float w) {
        Set(x, y, z, w);
    }

    Vector4(Vector4& v) {
        Set(v.x, v.y, v.z, v.w);
    }

#define VEC_MUTOP_V(op) \
    inline Vector4& \
    operator op (Vector4 v) {\
        Set(x op v.x, y op v.y, z op v.z, w op v.w);\
        return *this;\
    }

    VEC_MUTOP_V(+=)
    VEC_MUTOP_V(-=)
    VEC_MUTOP_V(*=)
    VEC_MUTOP_V(/=)

#undef VEC_MUTOP_V

    inline Vector4& 
    operator*=(float scale) {
        Set(x * scale, y * scale, z * scale, w * scale);
        return *this;
    }

    inline Vector4& 
    operator/=(float scale) {
        Set(x / scale, y / scale, z / scale, w / scale);
        return *this;
    }

    inline void
    Set(float x, float y, float z, float w) {
        this->x = x;
        this->y = y;
        this->z = z;
        this->w = w;
    }
};

inline bool
operator==(Vector4 a, Vector4 b) {
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}

inline bool
operator!=(Vector4 a, Vector4 b) {
    return !(a == b);
}

#define VEC_BINOP_V(op) \
inline Vector4 \
operator op (Vector4 a, Vector4 b) {\
    return Vector4(a.x op b.x, a.y op b.y, a.z op b.z, a.w op b.w);\
}

VEC_BINOP_V(+);
VEC_BINOP_V(-);
VEC_BINOP_V(*);
VEC_BINOP_V(/);
#undef VEC_BINOP_V

inline Vector4
operator*(Vector4 a, float b) {
    return Vector4(a.x * b, a.y * b, a.z * b, a.w * b);
}

inline Vector4
operator/(Vector4 a, float b) {
    return Vector4(a.x / b, a.y / b, a.z / b, a.w / b);
}

inline float 
Dot(Vector4 a, Vector4 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline float
Length(Vector4 a) {
    return sqrtf(Dot(a, a));
}

inline Vector4
Normalize(Vector4 a) {
    float length_sq = Dot(a, a);
    if (length_sq == 0.0f) {
        return a;
    }
    else {
        return a / sqrtf(length_sq);
    }
}

///////////////////////////////////////////////////////////////////////////////

struct Matrix22 {
    float e[4];

    Matrix22() {
        Set(0.0f, 0.0f,
            0.0f, 0.0f);
    }

    Matrix22(Matrix22& m) {
        Set(m(0, 0), m(0, 1),
            m(1, 0), m(1, 1));
    }

    Matrix22(float m00, float m01,
             float m10, float m11)
    {
        Set(m00, m01,
            m10, m11);
    }

    inline float& 
    operator()(size_t i, size_t j) {
        size_t idx = i*2+j;
        Assert(idx < 4, "Access out of bounds.");
        return e[idx];        
    }

    inline float 
    operator()(size_t i, size_t j) const {
        size_t idx = i*2+j;
        Assert(idx < 4, "Access out of bounds.");
        return e[idx];        
    }

    inline void
    Set(float m00, float m01,
        float m10, float m11)
    {
        e[0] = m00; e[1] = m01;
        e[2] = m10; e[3] = m11;
    }

    inline void
    SetIdentity() {
        Set(1.0f, 0.0f, 
            0.0f, 1.0f);
    }
};

inline bool
operator==(Matrix22 a, Matrix22 b) {
    for (u32 i = 0; i < 2; ++i) {
        for (u32 j = 0; j < 2; ++j) {
            if (a(i, j) != b(i, j)) {
                return false;
            }
        }
    }
    return true;
}

inline bool
Equals(Matrix22 a, Matrix22 b, float epsilon=1e-8) {
    for (u32 i = 0; i < 2; ++i) {
        for (u32 j = 0; j < 2; ++j) {
            if (fabsf(a(i, j) - b(i, j)) > epsilon) {
                return false;
            }
        }
    }
    return true;
}

inline bool
operator!=(Matrix22 a, Matrix22 b) {
    return !(a == b);
}

inline Matrix22
operator+(Matrix22 a, Matrix22 b) {
    return Matrix22(a(0, 0) + b(0, 0), a(0, 1) + b(0, 1),
                    a(1, 0) + b(1, 0), a(1, 1) + b(1, 1));
}

inline Matrix22
operator-(Matrix22 a, Matrix22 b) {
    return Matrix22(a(0, 0) - b(0, 0), a(0, 1) - b(0, 1),
                    a(1, 0) - b(1, 0), a(1, 1) - b(1, 1));
}

inline Matrix22
operator*(Matrix22 a, float b) {
    return Matrix22(a(0, 0) * b, a(0, 1) * b,
                    a(1, 0) * b, a(1, 1) * b);
}

inline Vector2
operator*(Matrix22 a, Vector2 b) {
    float x = a(0, 0) * b.x + a(0, 1) * b.y;
    float y = a(1, 0) * b.x + a(1, 1) * b.y;
    return Vector2(x, y);
}

inline Matrix22
operator*(Matrix22 a, Matrix22 b) {
    float m00 = a(0, 0) * b(0, 0)
              + a(0, 1) * b(1, 0);
    float m01 = a(0, 0) * b(0, 1)
              + a(0, 1) * b(1, 1);

    float m10 = a(1, 0) * b(0, 0)
              + a(1, 1) * b(1, 0);
    float m11 = a(1, 0) * b(0, 1)
              + a(1, 1) * b(1, 1);

    return Matrix22(m00, m01,
                    m10, m11);
}

inline Matrix22
Transpose(Matrix22 m) {
    return Matrix22(m(0, 0), m(1, 0),
                    m(0, 1), m(1, 1));
}

inline float
Determinant(Matrix22 m) {
    return m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0); 
}

inline Matrix22
Matrix22_Rotation(float angle) {
    float c = cosf(angle);
    float s = sinf(angle);
    return Matrix22(c, -s, 
                    s, c);
}

inline Matrix22
Invert(Matrix22 m) {
    float det = Determinant(m);
    Assert(det != 0.0f, "Cannot invert matrix with 0 determinant.");

    Matrix22 adj( m(1, 1), -m(0, 1),
                 -m(1, 0),  m(0, 0));

    return adj * (1.0f / det);
}

///////////////////////////////////////////////////////////////////////////////

struct Matrix33 {
    float e[9];
    
    Matrix33() {
        Set(0.0f, 0.0f, 0.0f, 
            0.0f, 0.0f, 0.0f, 
            0.0f, 0.0f, 0.0f);
    }

    Matrix33(Matrix33& m) {
        Set(m(0, 0), m(0, 1), m(0, 2), 
            m(1, 0), m(1, 1), m(1, 2), 
            m(2, 0), m(2, 1), m(2, 2));
    }

    Matrix33(float m00, float m01, float m02,
             float m10, float m11, float m12,
             float m20, float m21, float m22) 
    {
        Set(m00, m01, m02, 
            m10, m11, m12, 
            m20, m21, m22);
    }

    inline float& 
    operator()(size_t i, size_t j) {
        size_t idx = i*3+j;
        Assert(idx < 9, "Access out of bounds.");
        return e[idx];        
    }

    inline float 
    operator()(size_t i, size_t j) const {
        size_t idx = i*3+j;
        Assert(idx < 9, "Access out of bounds.");
        return e[idx];        
    }

    inline void 
    Set(size_t i, size_t j, float val) {
        (*this)(i, j) = val;
    }

    inline void 
    Set(float m00, float m01, float m02,
        float m10, float m11, float m12,
        float m20, float m21, float m22)
    {
        e[0] = m00; e[1] = m01; e[2] = m02;
        e[3] = m10; e[4] = m11; e[5] = m12;
        e[6] = m20; e[7] = m21; e[8] = m22;
    }

    inline void
    SetIdentity() {
        Set(1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f);
    }
};

inline bool
operator==(Matrix33 a, Matrix33 b) {
    for (u32 i = 0; i < 3; ++i) {
        for (u32 j = 0; j < 3; ++j) {
            if (a(i, j) != b(i, j)) {
                return false;
            }
        }
    }
    return true;
}

inline bool
operator!=(Matrix33 a, Matrix33 b) {
    return !(a == b);
}

inline bool
Equals(Matrix33 a, Matrix33 b, float epsilon=1e-8) {
    for (u32 i = 0; i < 3; ++i) {
        for (u32 j = 0; j < 3; ++j) {
            if (fabsf(a(i, j) - b(i, j)) > epsilon) {
                return false;
            }
        }
    }
    return true;
} 

inline Matrix33
operator+(Matrix33 a, Matrix33 b) {
    return Matrix33(
            a(0, 0) + b(0, 0), a(0, 1) + b(0, 1), a(0, 2) + b(0, 2),
            a(1, 0) + b(1, 0), a(1, 1) + b(1, 1), a(1, 2) + b(1, 2),
            a(2, 0) + b(2, 0), a(2, 1) + b(2, 1), a(2, 2) + b(2, 2)
        );
}

inline Matrix33
operator-(Matrix33 a, Matrix33 b) {
    return Matrix33(
            a(0, 0) - b(0, 0), a(0, 1) - b(0, 1), a(0, 2) - b(0, 2),
            a(1, 0) - b(1, 0), a(1, 1) - b(1, 1), a(1, 2) - b(1, 2),
            a(2, 0) - b(2, 0), a(2, 1) - b(2, 1), a(2, 2) - b(2, 2)
        );
}

inline Matrix33
operator*(Matrix33 a, float b) {
    return Matrix33(
            a(0, 0) * b, a(0, 1) * b, a(0, 2) * b,
            a(1, 0) * b, a(1, 1) * b, a(1, 2) * b,
            a(2, 0) * b, a(2, 1) * b, a(2, 2) * b
        );
}

inline Matrix33
operator*(Matrix33 a, Matrix33 b) {
    float m00 = a(0, 0) * b(0, 0)
              + a(0, 1) * b(1, 0)
              + a(0, 2) * b(2, 0);
    float m01 = a(0, 0) * b(0, 1)
              + a(0, 1) * b(1, 1)
              + a(0, 2) * b(2, 1);
    float m02 = a(0, 0) * b(0, 2)
              + a(0, 1) * b(1, 2)
              + a(0, 2) * b(2, 2);

    float m10 = a(1, 0) * b(0, 0)
              + a(1, 1) * b(1, 0)
              + a(1, 2) * b(2, 0);
    float m11 = a(1, 0) * b(0, 1)
              + a(1, 1) * b(1, 1)
              + a(1, 2) * b(2, 1);
    float m12 = a(1, 0) * b(0, 2)
              + a(1, 1) * b(1, 2)
              + a(1, 2) * b(2, 2);

    float m20 = a(2, 0) * b(0, 0)
              + a(2, 1) * b(1, 0)
              + a(2, 2) * b(2, 0);
    float m21 = a(2, 0) * b(0, 1)
              + a(2, 1) * b(1, 1)
              + a(2, 2) * b(2, 1);
    float m22 = a(2, 0) * b(0, 2)
              + a(2, 1) * b(1, 2)
              + a(2, 2) * b(2, 2);

    return Matrix33(
            m00, m01, m02,
            m10, m11, m12,
            m20, m21, m22
        );
}

inline Vector3
operator*(Matrix33 a, Vector3 b) {
    float x = a(0, 0) * b.x
            + a(0, 1) * b.y
            + a(0, 2) * b.z;

    float y = a(1, 0) * b.x
            + a(1, 1) * b.y
            + a(1, 2) * b.z;

    float z = a(2, 0) * b.x
            + a(2, 1) * b.y
            + a(2, 2) * b.z;

    return Vector3(x, y, z);
}

inline Matrix33
Transpose(Matrix33 m) {
    return Matrix33(
            m(0, 0), m(1, 0), m(2, 0),
            m(0, 1), m(1, 1), m(2, 1),
            m(0, 2), m(1, 2), m(2, 2)
        );
}

inline float
Determinant(Matrix33 m) {
    float a = m(0, 0);
    float b = m(0, 1);
    float c = m(0, 2);

    float d = m(1, 0);
    float e = m(1, 1);
    float f = m(1, 2);

    float g = m(2, 0);
    float h = m(2, 1);
    float i = m(2, 2);

    return a*e*i + b*f*g + c*d*h - c*e*g - b*d*i - a*f*h;
}

inline Matrix33
Matrix33_FromEuler(float heading, float altitude, float bank) {
    float ch = cosf(heading);
    float ca = cosf(altitude);
    float cb = cosf(bank);
    float sh = sinf(heading);
    float sa = sinf(altitude);
    float sb = sinf(bank);

    return Matrix33( ch*ca, -ch*sa*cb + sh*sb,  ch*sa*sb + sh*cb,
                     sa,     ca*cb,            -ca*sb,
                    -sh*ca,  sh*sa*cb + ch*sb, -sh*sa*sb + ch*cb);
}

static Matrix33
Matrix33_FromToRotation(Vector3 from, Vector3 to) {
    from = Normalize(from);
    to = Normalize(to);

    Vector3 cross = Cross(to, from);
    float st = Length(cross);
    float ct = Dot(from, to);

    const float epsilon = 1e-7;
    if (st < epsilon) {
        Matrix33 result;
        result.SetIdentity();
        return result;
    }

    Matrix33 rotation(ct,  -st,   0.0f,
                      st,   ct,   0.0f,
                      0.0f, 0.0f, 1.0f);

    Vector3 u = from;
    Vector3 v = Normalize(to - from * ct);
    Vector3 w = cross / st;

    Matrix33 basis(u.x, u.y, u.z,
                   v.x, v.y, v.z,
                   w.x, w.y, w.z);

    return Transpose(basis) * rotation * basis;
}
// TODO(bryan):  Want a proper "LookAt" function as well, that tries to 
// preserve up.  Can I express that with FromTo?

inline Matrix33
Invert(Matrix33 m) {
    float det = Determinant(m);
    Assert(det != 0.0f, "Cannot invert matrix with 0 determinant.");

    Matrix33 adj(
        m(1, 1)*m(2, 2) - m(1, 2)*m(2, 1), 
        m(0, 2)*m(2, 1) - m(0, 1)*m(2, 2), 
        m(0, 1)*m(1, 2) - m(0, 2)*m(1, 1),

        m(1, 2)*m(2, 0) - m(1, 0)*m(2, 2),
        m(0, 0)*m(2, 2) - m(0, 2)*m(2, 0),
        m(0, 2)*m(1, 0) - m(0, 0)*m(1, 2),

        m(1, 0)*m(2, 1) - m(1, 1)*m(2, 0),
        m(0, 1)*m(2, 0) - m(0, 0)*m(2, 1),
        m(0, 0)*m(1, 1) - m(0, 1)*m(1, 0)
    );

    return adj * (1.0f / det);
}

///////////////////////////////////////////////////////////////////////////////

struct Matrix44 {
    float e[16];
    
    Matrix44() {
        Set(0.0f, 0.0f, 0.0f, 0.0f, 
            0.0f, 0.0f, 0.0f, 0.0f, 
            0.0f, 0.0f, 0.0f, 0.0f, 
            0.0f, 0.0f, 0.0f, 0.0f);
    }

    Matrix44(Matrix44& m) {
        Set(m(0, 0), m(0, 1), m(0, 2), m(0,3), 
            m(1, 0), m(1, 1), m(1, 2), m(1,3), 
            m(2, 0), m(2, 1), m(2, 2), m(2,3),
            m(3, 0), m(3, 1), m(3, 2), m(3,3));
    }

    Matrix44(float m00, float m01, float m02, float m03,
             float m10, float m11, float m12, float m13,
             float m20, float m21, float m22, float m23, 
             float m30, float m31, float m32, float m33)
    {
        Set(m00, m01, m02, m03, 
            m10, m11, m12, m13, 
            m20, m21, m22, m23,
            m30, m31, m32, m33);
    }

    inline float& 
    operator()(size_t i, size_t j) {
        size_t idx = i*4+j;
        Assert(idx < 16, "Access out of bounds.");
        return e[idx];        
    }

    inline float 
    operator()(size_t i, size_t j) const {
        size_t idx = i*4+j;
        Assert(idx < 16, "Access out of bounds.");
        return e[idx];        
    }

    inline void 
    Set(size_t i, size_t j, float val) {
        (*this)(i, j) = val;
    }

    inline void 
    Set(float m00, float m01, float m02, float m03,
        float m10, float m11, float m12, float m13,
        float m20, float m21, float m22, float m23, 
        float m30, float m31, float m32, float m33)
    {
        e[ 0] = m00; e[ 1] = m01; e[ 2] = m02; e[ 3] = m03;
        e[ 4] = m10; e[ 5] = m11; e[ 6] = m12; e[ 7] = m13;
        e[ 8] = m20; e[ 9] = m21; e[10] = m22; e[11] = m23; 
        e[12] = m30; e[13] = m31; e[14] = m32; e[15] = m33;
    }

    inline void
    SetIdentity() {
        Set(1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f);
    }
};

inline Matrix44
operator+(Matrix44 a, Matrix44 b) {
    return Matrix44(
            a(0, 0) + b(0, 0), a(0, 1) + b(0, 1), a(0, 2) + b(0, 2), a(0, 3) + b(0, 3),
            a(1, 0) + b(1, 0), a(1, 1) + b(1, 1), a(1, 2) + b(1, 2), a(1, 3) + b(1, 3),
            a(2, 0) + b(2, 0), a(2, 1) + b(2, 1), a(2, 2) + b(2, 2), a(2, 3) + b(2, 3),
            a(3, 0) + b(3, 0), a(3, 1) + b(3, 1), a(3, 2) + b(3, 2), a(3, 3) + b(3, 3)
        );
}

inline Matrix44
operator-(Matrix44 a, Matrix44 b) {
    return Matrix44(
            a(0, 0) - b(0, 0), a(0, 1) - b(0, 1), a(0, 2) - b(0, 2), a(0, 3) - b(0, 3),
            a(1, 0) - b(1, 0), a(1, 1) - b(1, 1), a(1, 2) - b(1, 2), a(1, 3) - b(1, 3),
            a(2, 0) - b(2, 0), a(2, 1) - b(2, 1), a(2, 2) - b(2, 2), a(2, 3) - b(2, 3),
            a(3, 0) - b(3, 0), a(3, 1) - b(3, 1), a(3, 2) - b(3, 2), a(3, 3) - b(3, 3)
        );
}

inline Matrix44
operator*(Matrix44 a, float b) {
    return Matrix44(
            a(0, 0) * b, a(0, 1) * b, a(0, 2) * b, a(0, 3) * b,
            a(1, 0) * b, a(1, 1) * b, a(1, 2) * b, a(1, 3) * b,
            a(2, 0) * b, a(2, 1) * b, a(2, 2) * b, a(2, 3) * b,
            a(3, 0) * b, a(3, 1) * b, a(3, 2) * b, a(3, 3) * b
        );    
}

// TODO(bryan):  Even if we don't SIMD the rest of this stuff, may be worth
// switching over for these multiplies?
inline Matrix44 
operator*(Matrix44 a, Matrix44 b) {
    float m00 = a(0, 0) * b(0, 0)
              + a(0, 1) * b(1, 0)
              + a(0, 2) * b(2, 0)
              + a(0, 3) * b(3, 0);
    float m01 = a(0, 0) * b(0, 1)
              + a(0, 1) * b(1, 1)
              + a(0, 2) * b(2, 1)
              + a(0, 3) * b(3, 1);
    float m02 = a(0, 0) * b(0, 2)
              + a(0, 1) * b(1, 2)
              + a(0, 2) * b(2, 2)
              + a(0, 3) * b(3, 2);
    float m03 = a(0, 0) * b(0, 3)
              + a(0, 1) * b(1, 3)
              + a(0, 2) * b(2, 3)
              + a(0, 3) * b(3, 3);

    float m10 = a(1, 0) * b(0, 0)
              + a(1, 1) * b(1, 0)
              + a(1, 2) * b(2, 0)
              + a(1, 3) * b(3, 0);
    float m11 = a(1, 0) * b(0, 1)
              + a(1, 1) * b(1, 1)
              + a(1, 2) * b(2, 1)
              + a(1, 3) * b(3, 1);
    float m12 = a(1, 0) * b(0, 2)
              + a(1, 1) * b(1, 2)
              + a(1, 2) * b(2, 2)
              + a(1, 3) * b(3, 2);
    float m13 = a(1, 0) * b(0, 3)
              + a(1, 1) * b(1, 3)
              + a(1, 2) * b(2, 3)
              + a(1, 3) * b(3, 3);

    float m20 = a(2, 0) * b(0, 0)
              + a(2, 1) * b(1, 0)
              + a(2, 2) * b(2, 0)
              + a(2, 3) * b(3, 0);
    float m21 = a(2, 0) * b(0, 1)
              + a(2, 1) * b(1, 1)
              + a(2, 2) * b(2, 1)
              + a(2, 3) * b(3, 1);
    float m22 = a(2, 0) * b(0, 2)
              + a(2, 1) * b(1, 2)
              + a(2, 2) * b(2, 2)
              + a(2, 3) * b(3, 2);
    float m23 = a(2, 0) * b(0, 3)
              + a(2, 1) * b(1, 3)
              + a(2, 2) * b(2, 3)
              + a(2, 3) * b(3, 3);

    float m30 = a(3, 0) * b(0, 0)
              + a(3, 1) * b(1, 0)
              + a(3, 2) * b(2, 0)
              + a(3, 3) * b(3, 0);
    float m31 = a(3, 0) * b(0, 1)
              + a(3, 1) * b(1, 1)
              + a(3, 2) * b(2, 1)
              + a(3, 3) * b(3, 1);
    float m32 = a(3, 0) * b(0, 2)
              + a(3, 1) * b(1, 2)
              + a(3, 2) * b(2, 2)
              + a(3, 3) * b(3, 2);
    float m33 = a(3, 0) * b(0, 3)
              + a(3, 1) * b(1, 3)
              + a(3, 2) * b(2, 3)
              + a(3, 3) * b(3, 3);

    return Matrix44(
            m00, m01, m02, m03, 
            m10, m11, m12, m13, 
            m20, m21, m22, m23, 
            m30, m31, m32, m33 
        );
}

inline Vector4
operator*(Matrix44 a, Vector4 b) {
    float x = a(0, 0) * b.x
            + a(0, 1) * b.y
            + a(0, 2) * b.z
            + a(0, 3) * b.w;

    float y = a(1, 0) * b.x
            + a(1, 1) * b.y
            + a(1, 2) * b.z
            + a(1, 3) * b.w;

    float z = a(2, 0) * b.x
            + a(2, 1) * b.y
            + a(2, 2) * b.z
            + a(2, 3) * b.w;

    float w = a(3, 0) * b.x
            + a(3, 1) * b.y
            + a(3, 2) * b.z
            + a(3, 3) * b.w;

    return Vector4(x, y, z, w);
}

inline Matrix44
Transpose(Matrix44 m) {
    return Matrix44(
            m(0, 0), m(1, 0), m(2, 0), m(3, 0),
            m(0, 1), m(1, 1), m(2, 1), m(3, 1),
            m(0, 2), m(1, 2), m(2, 2), m(3, 2),
            m(0, 3), m(1, 3), m(2, 3), m(3, 3)
        );
}

inline Vector3
Matrix44_TransformVector(Matrix44 a, Vector3 b) {
    Vector4 bb = Vector4(b.x, b.y, b.z, 0.0f);
    bb = a * bb;
    return Vector3(bb.x, bb.y, bb.z);
}

inline Vector3
Matrix44_TransformPoint(Matrix44 a, Vector3 b) {
    Vector4 bb = Vector4(b.x, b.y, b.z, 1.0f);
    bb = a * bb;
    return Vector3(bb.x, bb.y, bb.z);    
}

///////////////////////////////////////////////////////////////////////////////

struct Quaternion {
    float w;
    float x;
    float y;
    float z;

    Quaternion() {
        Set(1.0f, 0.0f, 0.0f, 0.0f);
    }

    Quaternion(Quaternion& q) {
        Set(q.w, q.x, q.y, q.z);
    }

    Quaternion(float w, float x, float y, float z) {
        Set(w, x, y, z);
    }

    inline void
    Set(float w, float x, float y, float z) {
        this->w = w;
        this->x = x;
        this->y = y;
        this->z = z;
    }
};

inline bool
operator==(Quaternion a, Quaternion b) {
    return a.w == b.w && a.x == b.x && a.y == b.y && a.z == b.z;
}

inline bool
operator!=(Quaternion a, Quaternion b) {
    return !(a == b);
}

inline Quaternion
operator+(Quaternion a, Quaternion b) {
    return Quaternion(a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z);
}

inline Quaternion
operator-(Quaternion a, Quaternion b) {
    return Quaternion(a.w - b.w, a.x - b.x, a.y - b.y, a.z - b.z);
}

inline Quaternion
operator*(Quaternion a, float b) {
    return Quaternion(a.w * b, a.x * b, a.y * b, a.z * b);
}

inline Quaternion
operator/(Quaternion a, float b) {
    return Quaternion(a.w / b, a.x / b, a.y / b, a.z / b);
}


inline Quaternion
operator*(Quaternion a, Quaternion b) {
    return Quaternion(
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.x*b.w + a.w*b.x + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    );
}

inline float
Dot(Quaternion a, Quaternion b) {
    return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
}

inline float
Length(Quaternion q) {
    return sqrtf(Dot(q, q));
}

inline Quaternion 
Normalize(Quaternion q) {
    float length_sq = Dot(q, q);
    if (length_sq == 0.0f) {
        return Quaternion(0.0f, 0.0f, 0.0f, 0.0f);
    }
    else {
        float length = sqrtf(length_sq);
        return q / length;
    }
}

inline Quaternion
Quaternion_Inverse(Quaternion q) {
    float length_sq = Dot(q, q);
    if (length_sq == 0.0f) {
        return Quaternion(0.0f, 0.0f, 0.0f, 0.0f);
    }
    else {
        return Quaternion(q.w, -q.x, -q.y, -q.z) / length_sq;
    }
}

static Matrix33
Quaternion_ToMatrix(Quaternion q) {
    float xy = q.x*q.y;
    float xz = q.x*q.z;
    float xw = q.x*q.w;

    float yz = q.y*q.z;
    float yw = q.y*q.w;
    float zw = q.z*q.w;
    
    float xx = q.x*q.x;
    float yy = q.y*q.y;
    float zz = q.z*q.z;
    float ww = q.w*q.w;
    
    float m00 = 1.0f - 2.0f*(zz + yy);
    float m01 = 2.0f * (xy - zw);
    float m02 = 2.0f * (yw + xz);
    
    float m10 = 2.0f * (xy + zw);
    float m11 = 1.0f - 2.0f*(zz + xx);
    float m12 = 2.0f * (yz - xw);

    float m20 = 2.0f * (xz - yw);
    float m21 = 2.0f * (yz + xw);
    float m22 = 1.0f - 2.0f*(yy + xx); 

    return Matrix33(m00, m01, m02,
                    m10, m11, m12,
                    m20, m21, m22);
}

static Quaternion
Quaternion_FromEuler(float heading, float altitude, float bank) {
    float hh = heading * 0.5f;
    float ha = altitude * 0.5f;
    float hb = bank * 0.5f;

    float c1 = cosf(hh);
    float c2 = cosf(ha);
    float c3 = cosf(hb);
    float s1 = sinf(hh);
    float s2 = sinf(ha);
    float s3 = sinf(hb);
    
    float qw = c1*c2*c3 - s1*s2*s3;
    float qx = s1*s2*c3 + c1*c2*s3;
    float qy = s1*c2*c3 + c1*s2*s3;
    float qz = c1*s2*c3 - s1*c2*s3;

    return Quaternion(qw, qx, qy, qz);
}
