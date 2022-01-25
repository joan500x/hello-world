// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "AnimNode_SkeletalControlBase.h"
#include "CCDIK.h"
#include "AnimNode_CCDIK.generated.h"

/**
*	Controller which implements the CCDIK IK approximation algorithm
*/
USTRUCT()
struct ANIMGRAPHRUNTIME_API FAnimNode_CCDIK : public FAnimNode_SkeletalControlBase
{
	GENERATED_USTRUCT_BODY()

	/** Coordinates for target location of tip bone - if EffectorLocationSpace is bone, this is the offset from Target Bone to use as target location*/
	UPROPERTY(EditAnywhere, Category = Effector, meta = (PinShownByDefault))
	FVector EffectorLocation;

	/** Reference frame of Effector Transform. */
	UPROPERTY(EditAnywhere, Category = Effector)
	TEnumAsByte<enum EBoneControlSpace> EffectorLocationSpace;

	/** If EffectorTransformSpace is a bone, this is the bone to use. **/
	UPROPERTY(EditAnywhere, Category = Effector)
	FBoneSocketTarget EffectorTarget;

			/** Coordinates for target location of tip bone - if EffectorLocationSpace is bone, this is the offset from Target Bone to use as target location*/
			UPROPERTY(EditAnywhere, Category = Effector, meta = (PinShownByDefault))
				FVector EffectorLocation2;

			/** Reference frame of Effector Transform. */
			UPROPERTY(EditAnywhere, Category = Effector)
				TEnumAsByte<enum EBoneControlSpace> EffectorLocationSpace2;

			/** If EffectorTransformSpace is a bone, this is the bone to use. **/
			UPROPERTY(EditAnywhere, Category = Effector)
				FBoneSocketTarget EffectorTarget2;

	/** Name of tip bone */
	UPROPERTY(EditAnywhere, Category = Solver)
	FBoneReference TipBone;

	/** Name of the root bone*/
	UPROPERTY(EditAnywhere, Category = Solver)
	FBoneReference RootBone;

	/** Tolerance for final tip location delta from EffectorLocation*/
	UPROPERTY(EditAnywhere, Category = Solver)
	float Precision;

	/** Maximum number of iterations allowed, to control performance. */
	UPROPERTY(EditAnywhere, Category = Solver)
	int32 MaxIterations;

	/** Toggle drawing of axes to debug joint rotation*/
	UPROPERTY(EditAnywhere, Category = Solver)
	bool bStartFromTail;

	/** Tolerance for final tip location delta from EffectorLocation*/
	UPROPERTY(EditAnywhere, Category = Solver)
	bool bEnableRotationLimit;

	TArray<TArray<FCCDIKChainLink>> IKChainList;


	UWorld* m_MyWorld;
	USkeletalMeshComponent* m_SkelComp;
	TArray<FTransform> m_WorldSpaceBoneTM;
	FComponentSpacePoseContext* m_ComponentSpacePoseContext;
	int32 m_LastBoneIndicesCacheLOD = -1;
	TArray<int32> m_CachedBoneIndicesForCurrentLOD;
	TArray<FBoneTransform>* m_OutBoneTransforms;
	TArray<FBoneIndexType> m_InRagdollBones;
	bool m_UpdateBoneMapCrated = false;
	
	
	//these FBoneIndexTypes are bones of interest
	FBoneIndexType m_Spine02Bone;
	FBoneIndexType m_RightClavicleBone;
	FBoneIndexType m_RightHandBone;
	FBoneIndexType m_LeftThighBone;
	FBoneIndexType m_LeftFootBone;


private:
	/** symmetry rotation limit per joint. Index 0 matches with root bone and last index matches with tip bone. */
	UPROPERTY(EditAnywhere, EditFixedSize, Category = Solver)
	TArray<float> RotationLimitPerJoints;

				/** symmetry rotation limit per joint. Index 0 matches with root bone and last index matches with tip bone. */
				UPROPERTY(EditAnywhere, EditFixedSize, Category = Solver)
				TArray<float> RotationLimitPerJoints2;

public:
	FAnimNode_CCDIK();
	//FAnimNode_CCDIK(FVector _EffectorLocation, FBoneIndexType _TipBone, FBoneIndexType _RootBone);

	// FAnimNode_Base interface
	virtual void GatherDebugData(FNodeDebugData& DebugData) override;
	virtual bool HasPreUpdate() const { return true; }
	virtual void PreUpdate(const UAnimInstance* InAnimInstance) override;
	// End of FAnimNode_Base interface

	// FAnimNode_SkeletalControlBase interface
	virtual void EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms) override;
	virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;
	// End of FAnimNode_SkeletalControlBase interface

	//void GetWorldSpaceTransforms(TArray<FBoneTransform>& OutBoneTransforms);

	void GetComponentSpaceTransforms();

	TArray<FCCDIKChainLink> CreateIKChain(FBoneIndexType _TipBone, FBoneIndexType _RootBone);

	TArray<float> CreateRotationLimitArray(int32 NewSize);

	int32 CheckIfJointIsRelevantToChain(TArray<FCCDIKChainLink> CurrentChain, int32 CurrentBoneIndex, int32 CurrentChainIndex);

	int32 GetIndexFromRelevantChain(TArray<FCCDIKChainLink> CurrentChain, int32 CurrentBoneIndex);

	int32 GetIndexPosFromOutBoneTransforms(TArray<FBoneTransform> OutBoneTransforms, int32 CurrentBoneIndex);

	//void ApplyIKSolveBatch(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms);

	void DrawLine(FVector P1_in, FVector P2_in, FColor color, float thicknessMult);

	void printsmth();


private:
	// FAnimNode_SkeletalControlBase interface
	virtual void InitializeBoneReferences(const FBoneContainer& RequiredBones) override;
	// End of FAnimNode_SkeletalControlBase interface

	// Convenience function to get current (pre-translation iteration) component space location of bone by bone index
	FVector GetCurrentLocation(FCSPose<FCompactPose>& MeshBases, const FCompactPoseBoneIndex& BoneIndex);

	static FTransform GetTargetTransform(const FTransform& InComponentTransform, FCSPose<FCompactPose>& MeshBases, FBoneSocketTarget& InTarget, EBoneControlSpace Space, const FVector& InOffset);

public:
#if WITH_EDITOR
#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
	TArray<FVector> DebugLines;
#endif // #if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
	// resize rotation limit array based on set up
	void ResizeRotationLimitPerJoints(int32 NewSize);
#endif
};