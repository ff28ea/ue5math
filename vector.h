#pragma once
#include "ue4math.h"

struct FRotator;

struct FVector
{
public:
	double                                              X;                                            
	double                                              Y;                                                  
	double                                              Z;                                                   

	FVector() : X(0.0), Y(0.0), Z(0.0) {}
	FVector(double X, double Y, double Z) :X(X), Y(Y), Z(Z) {}

	double DotProduct(const FVector& v) const {
		return (X * v.X) + (Y * v.Y) + (Z * v.Z);
	}

	FVector CrossProduct(const FVector& v) const {
		FVector output;
		output.X = (Y * v.Z) - (Z * v.Y);
		output.Y = (Z * v.X) - (X * v.Z);
		output.Z = (X * v.Y) - (Y * v.X);
		return output;
	}

	FVector Min(const FVector& v) const {
		FVector output;
		output.X = X < v.X ? X : v.X;
		output.Y = Y < v.Y ? Y : v.Y;
		output.Z = Z < v.Z ? Z : v.Z;
		return output;
	}

	FVector Max(const FVector& v) const {
		FVector output;
		output.X = X > v.X ? X : v.X;
		output.Y = Y > v.Y ? Y : v.Y;
		output.Z = Z > v.Z ? Z : v.Z;
		return output;
	}

	bool operator == (const FVector& v) const {
		return X == v.X && Y == v.Y && Z == v.Z;
	}

	bool operator != (const FVector& v) const {
		return !(*this == v);
	}

	FVector operator - () const {
		return FVector(-X, -Y, -Z);
	}

	FVector operator + (const FVector& v) const {
		return FVector(X + v.X, Y + v.Y, Z + v.Z);
	}

	FVector operator - (const FVector& v) const {
		return FVector(X - v.X, Y - v.Y, Z - v.Z);
	}

	FVector operator * (const FVector& v) const {
		return FVector(X * v.X, Y * v.Y, Z * v.Z);
	}

	FVector operator * (double Value) const {
		return FVector(X * Value, Y * Value, Z * Value);
	}

	FVector GetNormalizedVector() const {
		return operator*(1.0 / sqrt(X * X + Y * Y + Z * Z));
	}

	void Normalize() {
		*this = GetNormalizedVector();
	}

	double Length() const {
		return sqrt(X * X + Y * Y + Z * Z);
	}

	double Distance(const FVector& v) const {
		return (v - *this).Length();
	}

	FVector operator ^ (const FVector& v) const {
		return CrossProduct(v);
	}

	double operator | (const FVector& v) const {
		return DotProduct(v);
	}

	FVector GetSignVector() const
	{
		return FVector
		(
			Select(X, 1.0, -1.0),
			Select(Y, 1.0, -1.0),
			Select(Z, 1.0, -1.0)
		);
	}

	bool IsNearlyZero(double Tolerance = KINDA_SMALL_NUMBER) const {
		return fabs(X) <= Tolerance && fabs(Y) <= Tolerance && fabs(Z) <= Tolerance;
	}

	FRotator GetDirectionRotator() const;
};

static FVector operator * (double Value, const FVector& v) {
	return v.operator*(Value);
}

static_assert(sizeof(FVector) == 24, "FVector");
class FVector2D
{
public:

	double                                              X;                                                         // 0x0000(0x0004) (Edit, BlueprintVisible, ZeroConstructor, SaveGame, IsPlainOldData, NoDestructor, HasGetValueTypeHash, NativeAccessSpecifierPublic)
	double                                              Y;                                                         // 0x0004(0x0004) (Edit, BlueprintVisible, ZeroConstructor, SaveGame, IsPlainOldData, NoDestructor, HasGetValueTypeHash, NativeAccessSpecifierPublic)

	inline FVector2D() : X(0), Y(0) {}

	inline FVector2D(double x, double y) : X(x), Y(y) {}
	inline bool Zero() const
	{
		return (X > -0.1 && X < 0.1 && Y > -0.1 && Y < 0.1);
	}
	inline FVector2D operator + (const FVector2D& other) const { return FVector2D(X + other.X, Y + other.Y); }

	inline FVector2D operator - (const FVector2D& other) const { return FVector2D(X - other.X, Y - other.Y); }

	inline FVector2D operator * (double scalar) const { return FVector2D(X * scalar, Y * scalar); }

	inline FVector2D operator * (const FVector2D& other) const { return FVector2D(X * other.X, Y * other.Y); }

	inline FVector2D operator / (double scalar) const { return FVector2D(X / scalar, Y / scalar); }

	inline FVector2D operator / (const FVector2D& other) const { return FVector2D(X / other.X, Y / other.Y); }

	inline FVector2D& operator=  (const FVector2D& other) { X = other.X; Y = other.Y; return *this; }

	inline FVector2D& operator+= (const FVector2D& other) { X += other.X; Y += other.Y; return *this; }

	inline FVector2D& operator-= (const FVector2D& other) { X -= other.X; Y -= other.Y; return *this; }

	inline FVector2D& operator*= (const double other) { X *= other; Y *= other; return *this; }
};