#pragma once
#include <math.h>

// =========================
// Fixed-size Vector4
// =========================

struct Row4 {
    float v[4];
    float& operator()(int i) { return v[i]; }
    float operator()(int i) const { return v[i]; }
};
struct Vec4 {
    float v[4];

    float& operator()(int i) { return v[i]; }
    float operator()(int i) const { return v[i]; }

    Vec4 operator+(const Vec4& other) const {
        Vec4 r;
        for(int i=0;i<4;i++) r.v[i] = v[i] + other.v[i];
        return r;
    }

    Vec4 operator-(const Vec4& other) const {
        Vec4 r;
        for(int i=0;i<4;i++) r.v[i] = v[i] - other.v[i];
        return r;
    }

    Vec4 operator*(float s) const {
        Vec4 r;
        for(int i=0;i<4;i++) r.v[i] = v[i] * s;
        return r;
    }
    
};


// =========================
// Fixed-size Matrix4x4
// =========================

struct Mat4 {
    float m[4][4];

    float* operator[](int r) { return m[r]; }
    const float* operator[](int r) const { return m[r]; }

    static Mat4 identity() {
        Mat4 I{};
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                I.m[i][j] = (i==j) ? 1.0f : 0.0f;
        return I;
    }

    Mat4 operator+(const Mat4& other) const {
        Mat4 r{};
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                r.m[i][j] = m[i][j] + other.m[i][j];
        return r;
    }

    Mat4 operator-(const Mat4& other) const {
        Mat4 r{};
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                r.m[i][j] = m[i][j] - other.m[i][j];
        return r;
    }
};

// Matrix multiplication 4x4 * 4x4
inline Mat4 mul(const Mat4& A, const Mat4& B) {
    Mat4 R{};
    for(int i=0;i<4;i++) {
        for(int j=0;j<4;j++) {
            float sum = 0.0f;
            for(int k=0;k<4;k++)
                sum += A.m[i][k] * B.m[k][j];
            R.m[i][j] = sum;
        }
    }
    return R;
}
// Outer product: Vec4 (column) × Row4 (row) = Mat4 (4x4)
inline Mat4 outerMul(const Vec4& k, const Row4& h) {
    Mat4 R{};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            R.m[i][j] = k.v[i] * h.v[j];
        }
    }
    return R;
}

// Matrix * Vector4
inline Vec4 mul(const Mat4& A, const Vec4& x) {
    Vec4 r{};
    for(int i=0;i<4;i++) {
        float sum = 0.0f;
        for(int j=0;j<4;j++)
            sum += A.m[i][j] * x.v[j];
        r.v[i] = sum;
    }
    return r;
}

// Transpose
inline Mat4 transpose(const Mat4& A) {
    Mat4 R{};
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            R.m[i][j] = A.m[j][i];
    return R;
}
inline Vec4 transpose(const Row4& r) {
        Vec4 v{};
        for (int i = 0; i < 4; i++)
            v.v[i] = r.v[i];
        return v;
}


// ===========================
// Row vector (1x4) and column vector (4x1)
// ===========================



// Row4 * Mat4 (1x4 * 4x4 = 1x4)
inline Row4 mul(const Row4& H, const Mat4& P) {
    Row4 r{};
    for(int j=0;j<4;j++) {
        float sum = 0.0f;
        for(int k=0;k<4;k++)
            sum += H.v[k] * P.m[k][j];
        r.v[j] = sum;
    }
    return r;
}

// Row4 * Vec4 (1x4 * 4x1 = scalar)
inline float dot(const Row4& H, const Vec4& x) {
    float sum = 0.0f;
    for(int i=0;i<4;i++) sum += H.v[i] * x.v[i];
    return sum;
}
