#pragma once
#include "ue4math.h"
#include "vector.h"
#include "quat.h"

struct FMatrix;


struct alignas(16) FTransform {
public:
	struct FQuat                                       Rotation;                                       
	struct FVector                                     Translation;                                           
private:	unsigned char                              UnknownData00[0x8];                                       
public:		struct FVector                             Scale3D;                                         

	FTransform();
	FTransform(const FQuat& Rotation, const FVector& Translation, const FVector& Scale3D);
	static bool AnyHasNegativeScale(const FVector& InScale3D, const FVector& InOtherScale3D);
	static void Multiply(FTransform* OutTransform, const FTransform* A, const FTransform* B);

	static void MultiplyUsingMatrixWithScale(FTransform* OutTransform, const FTransform* A, const FTransform* B);
	static void ConstructTransformFromMatrixWithDesiredScale(const FMatrix& AMatrix, const FMatrix& BMatrix, const FVector& DesiredScale, FTransform& OutTransform);

	FMatrix ToMatrixWithScale() const;

	FTransform operator*(const FTransform& A);

	static FVector GetSafeScaleReciprocal(const FVector& InScale, double Tolerance = SMALL_NUMBER);

	FVector GetBoneWithRotation(const FTransform& Bone) const;

	FTransform GetRelativeTransform(const FTransform& Other) const;

	FTransform Inverse();

	static void GetRelativeTransformUsingMatrixWithScale(FTransform* OutTransform, const FTransform* Base, const FTransform* Relative);
};

