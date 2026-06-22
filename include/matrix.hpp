#pragma once
/**
 * ============================================================
 *  Purpose : Self-contained fixed-size matrix library.
 *            No external dependencies — portable to any system.
 *
 *  Supports: Matrix<R,C>
 *    - element access  (r,c) and [i]
 *    - add, subtract, scalar multiply
 *    - matrix multiply
 *    - transpose
 *    - inverse (Gauss-Jordan with partial pivoting)
 *    - trace
 * ============================================================
 */

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <iomanip>
#include <sstream>

namespace auv {

template<int R, int C>
class Matrix {
public:
    std::array<double, R*C> data;

    Matrix() { data.fill(0.0); }

    // ── Element access ────────────────────────────────────────
    double& operator()(int r, int c){
        return data[r*C + c]; 
    }
    double  operator()(int r, int c) const { 
        return data[r*C + c]; 
    }
    double& operator[](int i){
        return data[i]; 
    }
    double  operator[](int i)const {
        return data[i]; 
    }

    // ── Static constructors ───────────────────────────────────
    static Matrix<R,C> Zero(){
        Matrix<R,C> m; m.data.fill(0.0);
        return m; 
    }
    static Matrix<R,C> Identity() {
        static_assert(R == C, "Identity requires square matrix");
        Matrix<R,C> m;
        for (int i = 0; i < R; ++i) m(i,i) = 1.0;
        return m;
    }

    // ── Arithmetic ────────────────────────────────────────────
    Matrix<R,C> operator+(const Matrix<R,C>& o) const {
        Matrix<R,C> out;
        for (int i = 0; i < R*C; ++i) out.data[i] = data[i] + o.data[i];
        return out;
    }
    Matrix<R,C>& operator+=(const Matrix<R,C>& o) {
        for (int i = 0; i < R*C; ++i) data[i] += o.data[i]; return *this;
    }
    Matrix<R,C> operator-(const Matrix<R,C>& o) const {
        Matrix<R,C> out;
        for (int i = 0; i < R*C; ++i) out.data[i] = data[i] - o.data[i];
        return out;
    }
    Matrix<R,C> operator*(double s) const {
        Matrix<R,C> out;
        for (int i = 0; i < R*C; ++i) out.data[i] = data[i] * s;
        return out;
    }
    Matrix<R,C>& operator*=(double s) {
        for (auto& v : data) v *= s; return *this;
    }
    Matrix<R,C> operator-() const { return (*this) * (-1.0); }

    // ── Matrix multiply  A(R×C) * B(C×K) = out(R×K) ─────────
    template<int K>
    Matrix<R,K> operator*(const Matrix<C,K>& B) const {
        Matrix<R,K> out;
        for (int r = 0; r < R; ++r)
            for (int k = 0; k < K; ++k) {
                double s = 0;
                for (int c = 0; c < C; ++c) s += (*this)(r,c) * B(c,k);
                out(r,k) = s;
            }
        return out;
    }

    // ── Transpose ─────────────────────────────────────────────
    Matrix<C,R> transpose() const {
        Matrix<C,R> out;
        for (int r = 0; r < R; ++r)
            for (int c = 0; c < C; ++c)
                out(c,r) = (*this)(r,c);
        return out;
    }

    // ── Inverse (Gauss-Jordan + partial pivot) ────────────────
    Matrix<R,C> inverse() const {
        static_assert(R == C, "Inverse requires square matrix");
        Matrix<R,C> A = *this;
        Matrix<R,C> I = Matrix<R,C>::Identity();
        for (int col = 0; col < R; ++col) {
            int pivot = col;
            for (int row = col+1; row < R; ++row)
                if (std::abs(A(row,col)) > std::abs(A(pivot,col))) pivot = row;
            if (std::abs(A(pivot,col)) < 1e-14)
                throw std::runtime_error("Matrix is singular — check noise params");
            for (int k = 0; k < R; ++k) {
                std::swap(A(col,k), A(pivot,k));
                std::swap(I(col,k), I(pivot,k));
            }
            double sc = A(col,col);
            for (int k = 0; k < R; ++k) { A(col,k) /= sc; I(col,k) /= sc; }
            for (int row = 0; row < R; ++row) {
                if (row == col) continue;
                double f = A(row,col);
                for (int k = 0; k < R; ++k) {
                    A(row,k) -= f*A(col,k);
                    I(row,k) -= f*I(col,k);
                }
            }
        }
        return I;
    }

    // ── Trace ─────────────────────────────────────────────────
    double trace() const {
        static_assert(R == C, "Trace requires square matrix");
        double t = 0; for (int i = 0; i < R; ++i) t += (*this)(i,i); return t;
    }

    // ── Clamp diagonal (keep P positive-definite) ─────────────
    void clampDiagonal(double min_val) {
        static_assert(R == C, "clampDiagonal requires square matrix");
        for (int i = 0; i < R; ++i)
            if ((*this)(i,i) < min_val) (*this)(i,i) = min_val;
    }

    // ── Debug print ───────────────────────────────────────────
    std::string str(int prec = 5) const {
        std::ostringstream oss; oss << std::fixed << std::setprecision(prec);
        for (int r = 0; r < R; ++r) {
            oss << "[ ";
            for (int c = 0; c < C; ++c) oss << std::setw(prec+5) << (*this)(r,c) << " ";
            oss << "]\n";
        }
        return oss.str();
    }
};

// ── Free: scalar * matrix ──────────────────────────────────────
template<int R, int C>
Matrix<R,C> operator*(double s, const Matrix<R,C>& m) { return m * s; }

// ── Type aliases (v2: 9-state system) ─────────────────────────
using Vec1   = Matrix<1,  1>;
using Vec2   = Matrix<2,  1>;
using Vec3   = Matrix<3,  1>;
using Vec9   = Matrix<9,  1>;
using Mat1   = Matrix<1,  1>;
using Mat2   = Matrix<2,  2>;
using Mat3   = Matrix<3,  3>;
using Mat9   = Matrix<9,  9>;
// Observation matrices: rows=meas_dim, cols=9
using Mat1x9 = Matrix<1,  9>;
using Mat2x9 = Matrix<2,  9>;
using Mat3x9 = Matrix<3,  9>;
// Kalman gain: rows=9, cols=meas_dim
using Mat9x1 = Matrix<9,  1>;
using Mat9x2 = Matrix<9,  2>;
using Mat9x3 = Matrix<9,  3>;

} 
