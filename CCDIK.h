// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "BoneIndices.h"
#include "DrawDebugHelpers.h"
#include "CCDIK.generated.h"

/** Transient structure for CCDIK node evaluation */
USTRUCT()
struct FCCDIKChainLink
{
	GENERATED_USTRUCT_BODY()

public:
	/** Transform of bone in component space. */
	FTransform Transform;

	/** Transform of bone in local space. This is mutable as their component space changes or parents*/
	FTransform LocalTransform;

	/** Transform Index that this control will output */
	int32 TransformIndex;

	FCompactPoseBoneIndex BoneIndex;

	/** Child bones which are overlapping this bone. 
	 * They have a zero length distance, so they will inherit this bone's transformation. */
	TArray<int32> ChildZeroLengthTransformIndices;

	float CurrentAngleDelta;

	FCCDIKChainLink()
		: TransformIndex(INDEX_NONE)
		, BoneIndex(INDEX_NONE)
		, CurrentAngleDelta(0.f)
	{
	}

	FCCDIKChainLink(const FTransform& InTransform, const FTransform& InLocalTransform, const int32& InTransformIndex, const FCompactPoseBoneIndex& _BoneIndex)
		: Transform(InTransform)
		, LocalTransform(InLocalTransform)
		, TransformIndex(InTransformIndex)
		, BoneIndex(_BoneIndex)//Añadido mio
		, CurrentAngleDelta(0.f)
	{
	}
};

namespace AnimationCore
{
	ANIMATIONCORE_API bool SolveCCDIK(TArray<FCCDIKChainLink>& ChainOfLinks, const FVector& TargetPosition, float Tolerance, float MaxNumIteration, bool bEnableRotationLimit, const TArray<float>& RotationLimitPerJoints, int32 CurrentBoneIndex);
};
