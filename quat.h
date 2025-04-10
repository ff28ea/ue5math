#pragma once
#include "ue4math.h"

struct FVector;
struct FMatrix;


struct alignas(16) FQuat {
public:
	double                                              X;                                          
	double                                              Y;                                                     
	double                                              Z;                                                   
	double                                              W;                                                  

	FQuat(double X = 0.0, double Y = 0.0, double Z = 0.0, double W = 1.0) : X(X), Y(Y), Z(Z), W(W) {}

	static void VectorQuaternionMultiply(void* Result, const void* Quat1, const void* Quat2) {
		typedef double double4[4];
		const double4& A = *((const double4*)Quat1);
		const double4& B = *((const double4*)Quat2);
		double4& R = *((double4*)Result);

		const double T0 = (A[2] - A[1]) * (B[1] - B[2]);
		const double T1 = (A[3] + A[0]) * (B[3] + B[0]);
		const double T2 = (A[3] - A[0]) * (B[1] + B[2]);
		const double T3 = (A[1] + A[2]) * (B[3] - B[0]);
		const double T4 = (A[2] - A[0]) * (B[0] - B[1]);
		const double T5 = (A[2] + A[0]) * (B[0] + B[1]);
		const double T6 = (A[3] + A[1]) * (B[3] - B[2]);
		const double T7 = (A[3] - A[1]) * (B[3] + B[2]);
		const double T8 = T5 + T6 + T7;
		const double T9 = 0.5f * (T4 + T8);

		R[0] = T1 + T9 - T8;
		R[1] = T2 + T9 - T7;
		R[2] = T3 + T9 - T6;
		R[3] = T0 + T9 - T5;
	}

	FQuat operator*(const FQuat& Q) const {
		FQuat Result;
		VectorQuaternionMultiply(&Result, this, &Q);
		return Result;
	}

	void Normalize(double Tolerance = SMALL_NUMBER)
	{
		const double SquareSum = X * X + Y * Y + Z * Z + W * W;

		if (SquareSum >= Tolerance)
		{
			const double Scale = InvSqrt(SquareSum);

			X *= Scale;
			Y *= Scale;
			Z *= Scale;
			W *= Scale;
		}
		else
		{
			*this = FQuat();
		}
	}

	double SizeSquared() const { return (X * X + Y * Y + Z * Z + W * W); }
	bool IsNormalized() const { return (fabs(1.0 - SizeSquared()) < THRESH_QUAT_NORMALIZED); }
	FQuat Inverse() const { return FQuat(-X, -Y, -Z, W); }

	FQuat(const FMatrix& M);

	FVector RotateVector(const FVector& V) const;
	FVector RotateVectorInverse(const FVector& V) const;
	FVector operator*(const FVector& V) const;
};
