#pragma once
#include "ue4math.h"

struct FVector;
struct FRotator;
struct FTransform;

struct FMatrix {
public:
    union
    {
        struct
        {
            double _11, _12, _13, _14;
            double _21, _22, _23, _24;
            double _31, _32, _33, _34;
            double _41, _42, _43, _44;
        };
        double M[4][4];
    };

    FMatrix() {
        //Identity matrix
        _11 = 1.0; _12 = 0.0; _13 = 0.0; _14 = 0.0;
        _21 = 0.0; _22 = 1.0; _23 = 0.0; _24 = 0.0;
        _31 = 0.0; _32 = 0.0; _33 = 1.0; _34 = 0.0;
        _41 = 0.0; _42 = 0.0; _43 = 0.0; _44 = 1.0;
    }

    FMatrix MatrixMultiply(const FMatrix& M2) const {
        const FMatrix& M1 = *this;
        FMatrix mResult;
        double x = M1.M[0][0];
        double y = M1.M[0][1];
        double z = M1.M[0][2];
        double w = M1.M[0][3];
        mResult.M[0][0] = (M2.M[0][0] * x) + (M2.M[1][0] * y) + (M2.M[2][0] * z) + (M2.M[3][0] * w);
        mResult.M[0][1] = (M2.M[0][1] * x) + (M2.M[1][1] * y) + (M2.M[2][1] * z) + (M2.M[3][1] * w);
        mResult.M[0][2] = (M2.M[0][2] * x) + (M2.M[1][2] * y) + (M2.M[2][2] * z) + (M2.M[3][2] * w);
        mResult.M[0][3] = (M2.M[0][3] * x) + (M2.M[1][3] * y) + (M2.M[2][3] * z) + (M2.M[3][3] * w);
        x = M1.M[1][0];
        y = M1.M[1][1];
        z = M1.M[1][2];
        w = M1.M[1][3];
        mResult.M[1][0] = (M2.M[0][0] * x) + (M2.M[1][0] * y) + (M2.M[2][0] * z) + (M2.M[3][0] * w);
        mResult.M[1][1] = (M2.M[0][1] * x) + (M2.M[1][1] * y) + (M2.M[2][1] * z) + (M2.M[3][1] * w);
        mResult.M[1][2] = (M2.M[0][2] * x) + (M2.M[1][2] * y) + (M2.M[2][2] * z) + (M2.M[3][2] * w);
        mResult.M[1][3] = (M2.M[0][3] * x) + (M2.M[1][3] * y) + (M2.M[2][3] * z) + (M2.M[3][3] * w);
        x = M1.M[2][0];
        y = M1.M[2][1];
        z = M1.M[2][2];
        w = M1.M[2][3];
        mResult.M[2][0] = (M2.M[0][0] * x) + (M2.M[1][0] * y) + (M2.M[2][0] * z) + (M2.M[3][0] * w);
        mResult.M[2][1] = (M2.M[0][1] * x) + (M2.M[1][1] * y) + (M2.M[2][1] * z) + (M2.M[3][1] * w);
        mResult.M[2][2] = (M2.M[0][2] * x) + (M2.M[1][2] * y) + (M2.M[2][2] * z) + (M2.M[3][2] * w);
        mResult.M[2][3] = (M2.M[0][3] * x) + (M2.M[1][3] * y) + (M2.M[2][3] * z) + (M2.M[3][3] * w);
        x = M1.M[3][0];
        y = M1.M[3][1];
        z = M1.M[3][2];
        w = M1.M[3][3];
        mResult.M[3][0] = (M2.M[0][0] * x) + (M2.M[1][0] * y) + (M2.M[2][0] * z) + (M2.M[3][0] * w);
        mResult.M[3][1] = (M2.M[0][1] * x) + (M2.M[1][1] * y) + (M2.M[2][1] * z) + (M2.M[3][1] * w);
        mResult.M[3][2] = (M2.M[0][2] * x) + (M2.M[1][2] * y) + (M2.M[2][2] * z) + (M2.M[3][2] * w);
        mResult.M[3][3] = (M2.M[0][3] * x) + (M2.M[1][3] * y) + (M2.M[2][3] * z) + (M2.M[3][3] * w);
        return mResult;
    }

    FMatrix operator * (const FMatrix& v) const { return MatrixMultiply(v); }

    void RemoveScaling(double Tolerance = SMALL_NUMBER)
    {
        // For each row, find magnitude, and if its non-zero re-scale so its unit length.
        const double SquareSum0 = (M[0][0] * M[0][0]) + (M[0][1] * M[0][1]) + (M[0][2] * M[0][2]);
        const double SquareSum1 = (M[1][0] * M[1][0]) + (M[1][1] * M[1][1]) + (M[1][2] * M[1][2]);
        const double SquareSum2 = (M[2][0] * M[2][0]) + (M[2][1] * M[2][1]) + (M[2][2] * M[2][2]);
        const double Scale0 = Select(SquareSum0 - Tolerance, InvSqrt(SquareSum0), 1.0);
        const double Scale1 = Select(SquareSum1 - Tolerance, InvSqrt(SquareSum1), 1.0);
        const double Scale2 = Select(SquareSum2 - Tolerance, InvSqrt(SquareSum2), 1.0);
        M[0][0] *= Scale0;
        M[0][1] *= Scale0;
        M[0][2] *= Scale0;
        M[1][0] *= Scale1;
        M[1][1] *= Scale1;
        M[1][2] *= Scale1;
        M[2][0] *= Scale2;
        M[2][1] *= Scale2;
        M[2][2] *= Scale2;
    }

    double Determinant() const
    {
        return	
            M[0][0] * (
                M[1][1] * (M[2][2] * M[3][3] - M[2][3] * M[3][2]) -
                M[2][1] * (M[1][2] * M[3][3] - M[1][3] * M[3][2]) +
                M[3][1] * (M[1][2] * M[2][3] - M[1][3] * M[2][2])
                ) -
            M[1][0] * (
                M[0][1] * (M[2][2] * M[3][3] - M[2][3] * M[3][2]) -
                M[2][1] * (M[0][2] * M[3][3] - M[0][3] * M[3][2]) +
                M[3][1] * (M[0][2] * M[2][3] - M[0][3] * M[2][2])
                ) +
            M[2][0] * (
                M[0][1] * (M[1][2] * M[3][3] - M[1][3] * M[3][2]) -
                M[1][1] * (M[0][2] * M[3][3] - M[0][3] * M[3][2]) +
                M[3][1] * (M[0][2] * M[1][3] - M[0][3] * M[1][2])
                ) -
            M[3][0] * (
                M[0][1] * (M[1][2] * M[2][3] - M[1][3] * M[2][2]) -
                M[1][1] * (M[0][2] * M[2][3] - M[0][3] * M[2][2]) +
                M[2][1] * (M[0][2] * M[1][3] - M[0][3] * M[1][2])
                );
    }

    FMatrix Inverse() const;

    void SetAxis0(const FVector& Axis);
    void SetAxis1(const FVector& Axis);
    void SetAxis2(const FVector& Axis);

    FVector GetOrigin() const;
    FVector GetScaledAxisX() const;
    FVector GetScaledAxisY() const;
    FVector GetScaledAxisZ() const;
    FRotator GetRotator() const;

    //Convert to FTransform
    FMatrix& operator=(const FTransform& t);
    FMatrix(const FTransform& t);
};