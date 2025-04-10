#include "ue4math.h"

#include "rotator.h"
#include "vector.h"
#include "quat.h"
#include "matrix.h"

FRotator::FRotator(const FQuat& q) {
	const double SingularitYTest = q.Z * q.X - q.W * q.Y;
	const double YawY = 2.0 * (q.W * q.Z + q.X * q.Y);
	const double YawX = (1.0 - 2.0 * (q.Y * q.Y + q.Z * q.Z));

	const double SINGULARITY_THRESHOLD = 0.4999995;
	const double RAD_TO_DEG = (180.0) / PI;

	if (SingularitYTest < -SINGULARITY_THRESHOLD) {
		Pitch = -90.0;
		Yaw = atan2(YawY, YawX) * RAD_TO_DEG;
		Roll = NormalizeAxis(-Yaw - (2.0 * atan2(q.X, q.W) * RAD_TO_DEG));
	}
	else if (SingularitYTest > SINGULARITY_THRESHOLD) {
		Pitch = 90.0;
		Yaw = atan2(YawY, YawX) * RAD_TO_DEG;
		Roll = NormalizeAxis(Yaw - (2.0 * atan2(q.X, q.W) * RAD_TO_DEG));
	}
	else {
		Pitch = asin(2.0 * (SingularitYTest)) * RAD_TO_DEG;
		Yaw = atan2(YawY, YawX) * RAD_TO_DEG;
		Roll = atan2(-2.0 * (q.W * q.X + q.Y * q.Z), (1.0 - 2.0 * (q.X * q.X + q.Y * q.Y))) * RAD_TO_DEG;
	}
}

FQuat FRotator::GetQuaternion() const {
	const double DEG_TO_RAD = PI / (180.0);
	const double RADS_DIVIDED_BY_2 = DEG_TO_RAD / 2.0;
	double SP, SY, SR;
	double CP, CY, CR;

	const double PitchNoWinding = fmod(Pitch, 360.0);
	const double YawNoWinding = fmod(Yaw, 360.0);
	const double RollNoWinding = fmod(Roll, 360.0);

	SP = sin(PitchNoWinding * RADS_DIVIDED_BY_2);
	CP = cos(PitchNoWinding * RADS_DIVIDED_BY_2);
	SY = sin(YawNoWinding * RADS_DIVIDED_BY_2);
	CY = cos(YawNoWinding * RADS_DIVIDED_BY_2);
	SR = sin(RollNoWinding * RADS_DIVIDED_BY_2);
	CR = cos(RollNoWinding * RADS_DIVIDED_BY_2);

	FQuat RotationQuat;
	RotationQuat.X = CR * SP * SY - SR * CP * CY;
	RotationQuat.Y = -CR * SP * CY - SR * CP * SY;
	RotationQuat.Z = CR * CP * SY - SR * SP * CY;
	RotationQuat.W = CR * CP * CY + SR * SP * SY;
	return RotationQuat;
}

FRotator::operator FQuat() const {
	return GetQuaternion();
}

FMatrix FRotator::GetMatrix(FVector origin) const {
	double radPitch = ConvertToRadians(Pitch);
	double radYaw = ConvertToRadians(Yaw);
	double radRoll = ConvertToRadians(Roll);

	double SP = sin(radPitch);
	double CP = cos(radPitch);
	double SY = sin(radYaw);
	double CY = cos(radYaw);
	double SR = sin(radRoll);
	double CR = cos(radRoll);

	FMatrix matriX;
	matriX.M[0][0] = CP * CY;
	matriX.M[0][1] = CP * SY;
	matriX.M[0][2] = SP;
	matriX.M[0][3] = 0.0;

	matriX.M[1][0] = SR * SP * CY - CR * SY;
	matriX.M[1][1] = SR * SP * SY + CR * CY;
	matriX.M[1][2] = -SR * CP;
	matriX.M[1][3] = 0.0;

	matriX.M[2][0] = -(CR * SP * CY + SR * SY);
	matriX.M[2][1] = CY * SR - CR * SP * SY;
	matriX.M[2][2] = CR * CP;
	matriX.M[2][3] = 0.0;

	matriX.M[3][0] = origin.X;
	matriX.M[3][1] = origin.Y;
	matriX.M[3][2] = origin.Z;
	matriX.M[3][3] = 1.0;

	return matriX;
}

FVector FRotator::GetUnitVector() const {
	double radPitch = ConvertToRadians(Pitch);
	double radYaw = ConvertToRadians(Yaw);

	double SP = sin(radPitch);
	double CP = cos(radPitch);
	double SY = sin(radYaw);
	double CY = cos(radYaw);

	return FVector(CP * CY, CP * SY, SP);
}


