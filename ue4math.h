#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>
#include <cstdint>
#include <cstddef>
#include <algorithm>

/*-----------------------------------------------------------------------------
	doubleing point constants.
-----------------------------------------------------------------------------*/
#define PI 					(3.1415926535897932)	/* Extra digits if needed: 3.1415926535897932384626433832795f */
#define SMALL_NUMBER		(1.e-8)
#define KINDA_SMALL_NUMBER	(1.e-4)
#define BIG_NUMBER			(3.4e+38)
#define EULERS_NUMBER       (2.71828182845904523536)
#define UE_GOLDEN_RATIO		(1.6180339887498948482045868343656381)	/* Also known as divine proportion, golden mean, or golden section - related to the Fibonacci Sequence = (1 + sqrt(5)) / 2 */
#define double_NON_FRACTIONAL (8388608.0) /* All single-precision doubleing point numbers greater than or equal to this have no fractional value. */

// Copied from double.h
#define MAX_FLT 3.402823466e+38

// Aux constants.
#define INV_PI			(0.31830988618f)
#define HALF_PI			(1.57079632679f)

// Common square roots
#define UE_SQRT_2		(1.4142135623730950488016887242097)
#define UE_SQRT_3		(1.7320508075688772935274463415059)
#define UE_INV_SQRT_2	(0.70710678118654752440084436210485)
#define UE_INV_SQRT_3	(0.57735026918962576450914878050196)
#define UE_HALF_SQRT_2	(0.70710678118654752440084436210485)
#define UE_HALF_SQRT_3	(0.86602540378443864676372317075294)


// Magic numbers for numerical precision.
#define DELTA			(0.00001)

/**
 * Lengths of normalized vectors (These are half their maximum values
 * to assure that dot products with normalized vectors don't overflow).
 */
#define double_NORMAL_THRESH				(0.0001)

 //
 // Magic numbers for numerical precision.
 //
#define THRESH_POINT_ON_PLANE			(0.10)		/* Thickness of plane for front/back/inside test */
#define THRESH_POINT_ON_SIDE			(0.20)		/* Thickness of polygon side's side-plane for point-inside/outside/on side test */
#define THRESH_POINTS_ARE_SAME			(0.00002)	/* Two points are same if within this distance */
#define THRESH_POINTS_ARE_NEAR			(0.015)	/* Two points are near if within this distance and can be combined if imprecise math is ok */
#define THRESH_NORMALS_ARE_SAME			(0.00002)	/* Two normal points are same if within this distance */
#define THRESH_UVS_ARE_SAME			    (0.0009765625)/* Two UV are same if within this threshold (1.0/1024f) */
													/* Making this too large results in incorrect CSG classification and disaster */
#define THRESH_VECTORS_ARE_NEAR			(0.0004)	/* Two vectors are near if within this distance and can be combined if imprecise math is ok */
													/* Making this too large results in lighting problems due to inaccurate texture coordinates */
#define THRESH_SPLIT_POLY_WITH_PLANE	(0.25)		/* A plane splits a polygon in half */
#define THRESH_SPLIT_POLY_PRECISELY		(0.01)		/* A plane exactly splits a polygon */
#define THRESH_ZERO_NORM_SQUARED		(0.0001)	/* Size of a unit normal that is considered "zero", squared */
#define THRESH_NORMALS_ARE_PARALLEL		(0.999845)	/* Two unit vectors are parallel if abs(A dot B) is greater than or equal to this. This is roughly cosine(1.0 degrees). */
#define THRESH_NORMALS_ARE_ORTHOGONAL	(0.017455)	/* Two unit vectors are orthogonal (perpendicular) if abs(A dot B) is less than or equal this. This is roughly cosine(89.0 degrees). */

#define THRESH_VECTOR_NORMALIZED		(0.01)		/** Allowed error for a normalized vector (against squared magnitude) */
#define THRESH_QUAT_NORMALIZED			(0.01)		/** Allowed error for a normalized quaternion (against squared magnitude) */

static double ConvertToRadians(double Degrees) { return Degrees * (PI / 180.0); }
static double ConvertToDegrees(double Radians) { return Radians * (180.0 / PI); }

static bool IsNearlyZero(double Value, double ErrorTolerance = 1.e-8)
{
	return fabs(Value) <= ErrorTolerance;
}

template< class T, class U >
static T Lerp(const T& A, const T& B, const U& Alpha)
{
	return (T)(A + Alpha * (B - A));
}

static double BezierInterp(double P0, double P1, double P2, double P3, double Alpha)
{
	const double P01 = Lerp(P0, P1, Alpha);
	const double P12 = Lerp(P1, P2, Alpha);
	const double P23 = Lerp(P2, P3, Alpha);
	const double P012 = Lerp(P01, P12, Alpha);
	const double P123 = Lerp(P12, P23, Alpha);
	const double P0123 = Lerp(P012, P123, Alpha);

	return P0123;
}

static void BezierToPower(double A1, double B1, double C1, double D1,
	double* A2, double* B2, double* C2, double* D2)
{
	double A = B1 - A1;
	double B = C1 - B1;
	double C = D1 - C1;
	double D = B - A;
	*A2 = C - B - D;
	*B2 = 3.0 * D;
	*C2 = 3.0 * A;
	*D2 = A1;
}

static int SolveCubic(double Coeff[4], double Solution[3])
{
	//auto cbrt = [](double x) -> double
	//{
	//	return ((x) > 0.0 ? pow((x), 1.0 / 3.0) : ((x) < 0.0 ? -pow((double)-(x), 1.0 / 3.0) : 0.0));
	//};
	int     NumSolutions = 0;

	/* normal form: x^3 + Ax^2 + Bx + C = 0 */

	double A = Coeff[2] / Coeff[3];
	double B = Coeff[1] / Coeff[3];
	double C = Coeff[0] / Coeff[3];

	/*  substitute x = y - A/3 to eliminate quadric term:
	x^3 +px + q = 0 */

	double SqOfA = A * A;
	double P = 1.0 / 3 * (-1.0 / 3 * SqOfA + B);
	double Q = 1.0 / 2 * (2.0 / 27 * A * SqOfA - 1.0 / 3 * A * B + C);

	/* use Cardano's formula */

	double CubeOfP = P * P * P;
	double D = Q * Q + CubeOfP;

	if (IsNearlyZero(D))
	{
		if (IsNearlyZero(Q)) /* one triple solution */
		{
			Solution[0] = 0;
			NumSolutions = 1;
		}
		else /* one single and one double solution */
		{
			double u = cbrt(-Q);
			Solution[0] = 2 * u;
			Solution[1] = -u;
			NumSolutions = 2;
		}
	}
	else if (D < 0) /* Casus irreducibilis: three real solutions */
	{
		double phi = 1.0 / 3 * acos(-Q / sqrt(-CubeOfP));
		double t = 2 * sqrt(-P);

		Solution[0] = t * cos(phi);
		Solution[1] = -t * cos(phi + PI / 3);
		Solution[2] = -t * cos(phi - PI / 3);
		NumSolutions = 3;
	}
	else /* one real solution */
	{
		double sqrt_D = sqrt(D);
		double u = cbrt(sqrt_D - Q);
		double v = -cbrt(sqrt_D + Q);

		Solution[0] = u + v;
		NumSolutions = 1;
	}

	/* resubstitute */

	double Sub = 1.0 / 3 * A;

	for (int i = 0; i < NumSolutions; ++i)
	{
		Solution[i] -= Sub;

	}
	return NumSolutions;
}

static double Select(double Comparand, double ValueGEZero, double ValueLTZero)
{
	return Comparand >= 0.0 ? ValueGEZero : ValueLTZero;
}

/** Computes a fully accurate inverse square root */
static double InvSqrt(double F)
{
	return 1.0 / sqrt(F);
}

/**
 * Calculate the inverse of an FMatrix.
 *
 * @param DstMatrix		FMatrix pointer to where the result should be stored
 * @param SrcMatrix		FMatrix pointer to the Matrix to be inversed
 */
static void VectorMatrixInverse(void* DstMatrix, const void* SrcMatrix)
{
	typedef double double4x4[4][4];
	const double4x4& M = *((const double4x4*)SrcMatrix);
	double4x4 Result;
	double Det[4];
	double4x4 Tmp;

	Tmp[0][0] = M[2][2] * M[3][3] - M[2][3] * M[3][2];
	Tmp[0][1] = M[1][2] * M[3][3] - M[1][3] * M[3][2];
	Tmp[0][2] = M[1][2] * M[2][3] - M[1][3] * M[2][2];

	Tmp[1][0] = M[2][2] * M[3][3] - M[2][3] * M[3][2];
	Tmp[1][1] = M[0][2] * M[3][3] - M[0][3] * M[3][2];
	Tmp[1][2] = M[0][2] * M[2][3] - M[0][3] * M[2][2];

	Tmp[2][0] = M[1][2] * M[3][3] - M[1][3] * M[3][2];
	Tmp[2][1] = M[0][2] * M[3][3] - M[0][3] * M[3][2];
	Tmp[2][2] = M[0][2] * M[1][3] - M[0][3] * M[1][2];

	Tmp[3][0] = M[1][2] * M[2][3] - M[1][3] * M[2][2];
	Tmp[3][1] = M[0][2] * M[2][3] - M[0][3] * M[2][2];
	Tmp[3][2] = M[0][2] * M[1][3] - M[0][3] * M[1][2];

	Det[0] = M[1][1] * Tmp[0][0] - M[2][1] * Tmp[0][1] + M[3][1] * Tmp[0][2];
	Det[1] = M[0][1] * Tmp[1][0] - M[2][1] * Tmp[1][1] + M[3][1] * Tmp[1][2];
	Det[2] = M[0][1] * Tmp[2][0] - M[1][1] * Tmp[2][1] + M[3][1] * Tmp[2][2];
	Det[3] = M[0][1] * Tmp[3][0] - M[1][1] * Tmp[3][1] + M[2][1] * Tmp[3][2];

	double Determinant = M[0][0] * Det[0] - M[1][0] * Det[1] + M[2][0] * Det[2] - M[3][0] * Det[3];
	const double	RDet = 1.0 / Determinant;

	Result[0][0] = RDet * Det[0];
	Result[0][1] = -RDet * Det[1];
	Result[0][2] = RDet * Det[2];
	Result[0][3] = -RDet * Det[3];
	Result[1][0] = -RDet * (M[1][0] * Tmp[0][0] - M[2][0] * Tmp[0][1] + M[3][0] * Tmp[0][2]);
	Result[1][1] = RDet * (M[0][0] * Tmp[1][0] - M[2][0] * Tmp[1][1] + M[3][0] * Tmp[1][2]);
	Result[1][2] = -RDet * (M[0][0] * Tmp[2][0] - M[1][0] * Tmp[2][1] + M[3][0] * Tmp[2][2]);
	Result[1][3] = RDet * (M[0][0] * Tmp[3][0] - M[1][0] * Tmp[3][1] + M[2][0] * Tmp[3][2]);
	Result[2][0] = RDet * (
		M[1][0] * (M[2][1] * M[3][3] - M[2][3] * M[3][1]) -
		M[2][0] * (M[1][1] * M[3][3] - M[1][3] * M[3][1]) +
		M[3][0] * (M[1][1] * M[2][3] - M[1][3] * M[2][1])
		);
	Result[2][1] = -RDet * (
		M[0][0] * (M[2][1] * M[3][3] - M[2][3] * M[3][1]) -
		M[2][0] * (M[0][1] * M[3][3] - M[0][3] * M[3][1]) +
		M[3][0] * (M[0][1] * M[2][3] - M[0][3] * M[2][1])
		);
	Result[2][2] = RDet * (
		M[0][0] * (M[1][1] * M[3][3] - M[1][3] * M[3][1]) -
		M[1][0] * (M[0][1] * M[3][3] - M[0][3] * M[3][1]) +
		M[3][0] * (M[0][1] * M[1][3] - M[0][3] * M[1][1])
		);
	Result[2][3] = -RDet * (
		M[0][0] * (M[1][1] * M[2][3] - M[1][3] * M[2][1]) -
		M[1][0] * (M[0][1] * M[2][3] - M[0][3] * M[2][1]) +
		M[2][0] * (M[0][1] * M[1][3] - M[0][3] * M[1][1])
		);
	Result[3][0] = -RDet * (
		M[1][0] * (M[2][1] * M[3][2] - M[2][2] * M[3][1]) -
		M[2][0] * (M[1][1] * M[3][2] - M[1][2] * M[3][1]) +
		M[3][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
		);
	Result[3][1] = RDet * (
		M[0][0] * (M[2][1] * M[3][2] - M[2][2] * M[3][1]) -
		M[2][0] * (M[0][1] * M[3][2] - M[0][2] * M[3][1]) +
		M[3][0] * (M[0][1] * M[2][2] - M[0][2] * M[2][1])
		);
	Result[3][2] = -RDet * (
		M[0][0] * (M[1][1] * M[3][2] - M[1][2] * M[3][1]) -
		M[1][0] * (M[0][1] * M[3][2] - M[0][2] * M[3][1]) +
		M[3][0] * (M[0][1] * M[1][2] - M[0][2] * M[1][1])
		);
	Result[3][3] = RDet * (
		M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
		M[1][0] * (M[0][1] * M[2][2] - M[0][2] * M[2][1]) +
		M[2][0] * (M[0][1] * M[1][2] - M[0][2] * M[1][1])
		);

	memcpy(DstMatrix, &Result, 16 * sizeof(double));
}