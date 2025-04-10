#pragma once
#include "ue4math.h"
#include "vector.h"

struct FQuat;
struct FMatrix;

// ScriptStruct CoreUObject.Rotator
// 0x000C
struct FRotator {
public:
	double                                              Pitch;                                                    // 0x0000(0x0004) (CPF_Edit, CPF_BlueprintVisible, CPF_ZeroConstructor, CPF_SaveGame, CPF_IsPlainOldData)
	double                                              Yaw;                                                      // 0x0004(0x0004) (CPF_Edit, CPF_BlueprintVisible, CPF_ZeroConstructor, CPF_SaveGame, CPF_IsPlainOldData)
	double                                              Roll;                                                     // 0x0008(0x0004) (CPF_Edit, CPF_BlueprintVisible, CPF_ZeroConstructor, CPF_SaveGame, CPF_IsPlainOldData)

	FRotator() : Pitch(0.0), Yaw(0.0), Roll(0.0) {}
	FRotator(double pitch, double yaw, double roll) : Pitch(pitch), Yaw(yaw), Roll(roll) {}

	static double NormalizeAxis(double Angle) {
		//-180 ~ 180
		if (Angle > 180.0)
			Angle -= 360.0;
		if (Angle < -180.0)
			Angle += 360.0;
		return Angle;
	}

	void Clamp() {
		Pitch = std::clamp(NormalizeAxis(Pitch), -75.0, 75.0);
		Yaw = NormalizeAxis(Yaw);
		Roll = NormalizeAxis(Roll);
	}

	double InnerProduct(const FRotator& v) const {
		return (Pitch * v.Pitch) + (Yaw * v.Yaw) + (Roll * v.Roll);
	}

	FRotator OuterProduct(const FRotator& v) const {
		FRotator output;
		output.Pitch = (Yaw * v.Roll) - (Roll * v.Yaw);
		output.Yaw = (Roll * v.Pitch) - (Pitch * v.Roll);
		output.Roll = (Pitch * v.Yaw) - (Yaw * v.Pitch);
		return output;
	}

	bool operator == (const FRotator& v) const {
		return Pitch == v.Pitch && Yaw == v.Yaw && Roll == v.Roll;
	}

	bool operator != (const FRotator& v) const {
		return !(*this == v);
	}

	FRotator operator - () const {
		return FRotator(-Pitch, -Yaw, -Roll);
	}

	FRotator operator + (const FRotator& v) const {
		return FRotator(Pitch + v.Pitch, Yaw + v.Yaw, Roll + v.Roll);
	}

	FRotator operator - (const FRotator& v) const {
		return FRotator(Pitch - v.Pitch, Yaw - v.Yaw, Roll - v.Roll);
	}

	FRotator operator * (double Value) const {
		return FRotator(Pitch * Value, Yaw * Value, Roll * Value);
	}

	double Length() const {
		return sqrt(Pitch * Pitch + Yaw * Yaw + Roll * Roll);
	}

	double Distance(const FRotator& v) const {
		return (v - *this).Length();
	}

	FRotator operator ^ (const FRotator& v) const {
		return OuterProduct(v);
	}

	double operator * (const FRotator& v) const {
		return InnerProduct(v);
	}

	FQuat GetQuaternion() const;
	FRotator(const FQuat& q);
	operator FQuat() const;

	FVector GetUnitVector() const;
	FMatrix GetMatrix(FVector origin = { 0, 0, 0 }) const;
};
