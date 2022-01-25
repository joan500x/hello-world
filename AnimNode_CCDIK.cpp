// Copyright Epic Games, Inc. All Rights Reserved.

#include "BoneControllers/AnimNode_CCDIK.h"
#include "Animation/AnimTypes.h"
#include "AnimationRuntime.h"
#include "DrawDebugHelpers.h"
#include "Animation/AnimInstanceProxy.h"

/////////////////////////////////////////////////////
// AnimNode_CCDIK
// Implementation of the CCDIK IK Algorithm

FAnimNode_CCDIK::FAnimNode_CCDIK()
	: EffectorLocation(FVector::ZeroVector)
	, EffectorLocationSpace(BCS_ComponentSpace)
	, Precision(1.f)
	, MaxIterations(10)
	, bStartFromTail(true)
	, bEnableRotationLimit(false)
{
}

//FAnimNode_CCDIK::FAnimNode_CCDIK(FVector _EffectorLocation, FBoneIndexType _TipBone, FBoneIndexType _RootBone)
//{
//	EffectorLocation = _EffectorLocation;
//	TipBone = _TipBone;
//	RootBone = _RootBone;
//	TArray<FCCDIKChainLink> IKChain = CreateIKChain(TipBone, RootBone);
//}

FVector FAnimNode_CCDIK::GetCurrentLocation(FCSPose<FCompactPose>& MeshBases, const FCompactPoseBoneIndex& BoneIndex)
{
	return MeshBases.GetComponentSpaceTransform(BoneIndex).GetLocation();
}

FTransform FAnimNode_CCDIK::GetTargetTransform(const FTransform& InComponentTransform, FCSPose<FCompactPose>& MeshBases, FBoneSocketTarget& InTarget, EBoneControlSpace Space, const FVector& InOffset)
{
	FTransform OutTransform;
	if (Space == BCS_BoneSpace)
	{
		OutTransform = InTarget.GetTargetTransform(InOffset, MeshBases, InComponentTransform);
	}
	else
	{
		// parent bone space still goes through this way
		// if your target is socket, it will try find parents of joint that socket belongs to
		OutTransform.SetLocation(InOffset);
		FAnimationRuntime::ConvertBoneSpaceTransformToCS(InComponentTransform, MeshBases, OutTransform, InTarget.GetCompactPoseBoneIndex(), Space);
	}

	return OutTransform;
}

//FUNCTION CREATED BY ME
void FAnimNode_CCDIK::PreUpdate(const UAnimInstance* InAnimInstance)
{
	m_MyWorld = InAnimInstance->GetSkelMeshComponent()->GetWorld();
	m_SkelComp = InAnimInstance->GetSkelMeshComponent();

	if (!m_SkelComp || !m_SkelComp->GetWorld())
	{
		return;
	}

	// Determine the bones that need to be updated (only do this once)
	if (!m_UpdateBoneMapCrated)
	{
		// Then for example, here’s one way to get references to all bones in the PhysicsAsset (ragdoll structure) by name.  You don’t necessarily need to grab from the PhysicsAsset, but it could be one way to get a grasp of the whole body.  
		//Then you could essentially use the PhsicsAsset to define all the bones in the body that your full body IK algorithm might be interested in.
		for (int32 iBone = 0; iBone < InAnimInstance->GetSkelMeshComponent()->GetPhysicsAsset()->SkeletalBodySetups.Num(); iBone++)
		{
			int32 PhysBoneIndex = m_SkelComp->SkeletalMesh->RefSkeleton.FindBoneIndex(InAnimInstance->GetSkelMeshComponent()->GetPhysicsAsset()->SkeletalBodySetups[iBone]->BoneName);
			if (PhysBoneIndex != INDEX_NONE)
			{
				m_InRagdollBones.Add(PhysBoneIndex);
			}
		}
		m_InRagdollBones.Sort();

		// Additionally, you could also note specific bones of interest for whatever reason
		FName spine_02("spine_02");
		FName hand_r("hand_r");
		FName clavicle_r("clavicle_r");
		FName thigh_l("thigh_l");
		FName foot_l("foot_l");

		{
			TArray<FName> BoneNames;
			m_SkelComp->GetBoneNames(BoneNames);
			

			m_RightClavicleBone = m_SkelComp->SkeletalMesh->RefSkeleton.FindBoneIndex(clavicle_r);
			m_Spine02Bone = m_SkelComp->SkeletalMesh->RefSkeleton.FindBoneIndex(spine_02);
			m_RightHandBone = m_SkelComp->SkeletalMesh->RefSkeleton.FindBoneIndex(hand_r);
			m_LeftThighBone = m_SkelComp->SkeletalMesh->RefSkeleton.FindBoneIndex(thigh_l);
			m_LeftFootBone = m_SkelComp->SkeletalMesh->RefSkeleton.FindBoneIndex(foot_l);
		}
		m_UpdateBoneMapCrated = true;
	}

}

////FUNCTION CREATED BY ME
//void FAnimNode_CCDIK::GetWorldSpaceTransforms(TArray<FBoneTransform>& OutBoneTransforms)
//{
//	const FTransform CompWorldSpaceTM = m_ComponentSpacePoseContext->AnimInstanceProxy->GetComponentTransform();
//
//	OutBoneTransforms.AddUninitialized(m_SkelComp->Bodies.Num());
//
//	for (int32 iJoint = 0; iJoint < m_SkelComp->Bodies.Num(); iJoint++)
//	{
//		// Get the UE part transforms from skeleton here as in USkeletalMeshComponent::UpdateKinematicBonesToAnim (just locally)
//		FCompactPoseBoneIndex BoneIndex = FCompactPoseBoneIndex(m_CachedBoneIndicesForCurrentLOD[iJoint]);
//		const FTransform& ComponentSpaceTM = m_ComponentSpacePoseContext->Pose.GetComponentSpaceTransform(BoneIndex);
//		const FTransform WorldSpaceBoneTM = ComponentSpaceTM * CompWorldSpaceTM;
//
//		m_WorldSpaceBoneTM[iJoint] = WorldSpaceBoneTM;
//		OutBoneTransforms[iJoint] = FBoneTransform(BoneIndex, WorldSpaceBoneTM);
//	}
//}

//FUNCTION CREATED BY ME
void FAnimNode_CCDIK::GetComponentSpaceTransforms()
{
	m_OutBoneTransforms->AddUninitialized(m_SkelComp->Bodies.Num());

	for (int32 iJoint = 0; iJoint < m_SkelComp->Bodies.Num(); iJoint++)
	{
		// Get the UE part transforms from skeleton here as in USkeletalMeshComponent::UpdateKinematicBonesToAnim (just locally)

		FCompactPoseBoneIndex BoneIndex = FCompactPoseBoneIndex(m_CachedBoneIndicesForCurrentLOD[iJoint]);
		//FCompactPoseBoneIndex BoneIndex = FCompactPoseBoneIndex(iJoint);
		const FTransform& ComponentSpaceTM = m_ComponentSpacePoseContext->Pose.GetComponentSpaceTransform(BoneIndex);
		ComponentSpaceTM.DiagnosticCheck_IsValid();

		(*m_OutBoneTransforms)[iJoint] = FBoneTransform(BoneIndex, ComponentSpaceTM);
	}
}

//FUNCTION CREATED BY ME
TArray<FCCDIKChainLink> FAnimNode_CCDIK::CreateIKChain(FBoneIndexType TipLink, FBoneIndexType RootLink)
{
	const FBoneContainer& BoneContainer = m_ComponentSpacePoseContext->Pose.GetPose().GetBoneContainer();

	// Gather all bone indices between root and tip.
	TArray<FCompactPoseBoneIndex> BoneIndices;

	{	//Rootbone and TipBone index declaration
		const FCompactPoseBoneIndex RootIndex = FCompactPoseBoneIndex(RootLink);
		FCompactPoseBoneIndex BoneIndex = FCompactPoseBoneIndex(TipLink);
		//Fill the array by inserting one by one the indices of the bones in the 0 position and moving the previous ones to the right, until you reach the rootbone.
		do
		{
			BoneIndices.Insert(BoneIndex, 0);
			BoneIndex = m_ComponentSpacePoseContext->Pose.GetPose().GetParentBoneIndex(BoneIndex);
		} while (BoneIndex != RootIndex);
		BoneIndices.Insert(BoneIndex, 0);
	}

	//Gather transforms
	int32 const NumTransforms = BoneIndices.Num();

	// Gather chain links and initialize Chain array. These are non zero length bones.
	TArray<FCCDIKChainLink> Chain;
	Chain.Reserve(NumTransforms);

	//Start filling the arrays with Root Bone
	{
		const FCompactPoseBoneIndex& RootBoneIndex = BoneIndices[0];
		const FTransform& LocalTransform = m_ComponentSpacePoseContext->Pose.GetLocalSpaceTransform(RootBoneIndex);
		const FTransform& BoneCSTransform = m_ComponentSpacePoseContext->Pose.GetComponentSpaceTransform(RootBoneIndex);

		//Add Rootbone to OutBoneTransforms array and Chain array in position 0
		Chain.Add(FCCDIKChainLink(BoneCSTransform, LocalTransform, 0, RootBoneIndex));
	}

	// Go through remaining transforms
	for (int32 TransformIndex = 1; TransformIndex < NumTransforms; TransformIndex++)
	{
		//Get the Bone Index
		const FCompactPoseBoneIndex& BoneIndex = BoneIndices[TransformIndex];

		//Get CS and Local Transforms for that BoneIndex
		const FTransform& LocalTransform = m_ComponentSpacePoseContext->Pose.GetLocalSpaceTransform(BoneIndex);
		const FTransform& BoneCSTransform = m_ComponentSpacePoseContext->Pose.GetComponentSpaceTransform(BoneIndex);
		FVector const BoneCSPosition = BoneCSTransform.GetLocation();

		Chain.Add(FCCDIKChainLink(BoneCSTransform, LocalTransform, TransformIndex, BoneIndex));
	}

	IKChainList.Add(Chain);

	return Chain;
}

TArray<float> FAnimNode_CCDIK::CreateRotationLimitArray(int32 Size)
{
	TArray<float> RotationLimitArray;
	RotationLimitArray.Reserve(Size);
	for (int32 i = 0; i < Size; i++)
	{
		RotationLimitArray.Add(50.f);
	}
	return RotationLimitArray;

}



//FUNCTION CREATED BY ME
int32 FAnimNode_CCDIK::CheckIfJointIsRelevantToChain(TArray<FCCDIKChainLink> CurrentChain, int32 CurrentBoneIndex, int32 CurrentChainIndex)
{
	int32 SizeOfChain = CurrentChain.Num();
	for (int32 iLink = 0; iLink < SizeOfChain; iLink++)
	{
		if (CurrentBoneIndex == CurrentChain[iLink].BoneIndex.GetInt())
		{
			return CurrentChainIndex;
		}
	}
	return -1;
}


//FUNCTION CREATED BY ME
int32 FAnimNode_CCDIK::GetIndexFromRelevantChain(TArray<FCCDIKChainLink> CurrentChain, int32 CurrentBoneIndex)
{
	int32 SizeOfChain = CurrentChain.Num();
	for (int32 iLink = 0; iLink < SizeOfChain; iLink++)
	{
		if (CurrentBoneIndex == CurrentChain[iLink].BoneIndex.GetInt())
		{
			return iLink;
		}
	}
	return -1;
}

//FUNCTION CREATED BY ME
int32 FAnimNode_CCDIK::GetIndexPosFromOutBoneTransforms(TArray<FBoneTransform> OutBoneTransforms, int32 CurrentBoneIndex)
{
	int32 SizeOfChain = OutBoneTransforms.Num();
	for (int32 iIndexPos = 0; iIndexPos < SizeOfChain; iIndexPos++)
	{
		if (CurrentBoneIndex == OutBoneTransforms[iIndexPos].BoneIndex.GetInt())
		{
			return iIndexPos;
		}
	}
	return -1;
}


//FUNCTION CREATED BY ME
//CCDIK UPDATE
//void FAnimNode_CCDIK::ApplyIKSolveBatch(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
void FAnimNode_CCDIK::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	DECLARE_SCOPE_HIERARCHICAL_COUNTER_ANIMNODE(EvaluateSkeletalControl_AnyThread)
	const FBoneContainer& BoneContainer = Output.Pose.GetPose().GetBoneContainer();

	// Store references to the context and out transforms
	m_ComponentSpacePoseContext = &Output;
	m_OutBoneTransforms = &OutBoneTransforms;

	//If the LOD has changed, re-cache link bones
	if (m_SkelComp->PredictedLODLevel != m_LastBoneIndicesCacheLOD)
	{
		const int32 NumBodies = m_SkelComp->Bodies.Num();
		if (m_LastBoneIndicesCacheLOD == -1)
		{
			m_CachedBoneIndicesForCurrentLOD.SetNum(NumBodies);
		}

		m_LastBoneIndicesCacheLOD = m_SkelComp->PredictedLODLevel;

		const auto& BoneIndices = BoneContainer.GetBoneIndicesArray();
		for (int32 iLinkOnCBR = 0; iLinkOnCBR < NumBodies; iLinkOnCBR++)
		{
			FBodyInstance* BodyInst = m_SkelComp->Bodies[iLinkOnCBR];
			const int32 BoneIndex = BodyInst->InstanceBoneIndex;
			m_CachedBoneIndicesForCurrentLOD[iLinkOnCBR] = BoneIndices.Find(BoneIndex);
		}
	}

	GetComponentSpaceTransforms();

	int32 NumBones = m_CachedBoneIndicesForCurrentLOD.Num();
	//int32 NumBones = BoneContainer.GetNumBones();


	// Update EffectorLocation if it is based off a bone position
	FTransform CSEffectorTransform = GetTargetTransform(Output.AnimInstanceProxy->GetComponentTransform(), Output.Pose, EffectorTarget, EffectorLocationSpace, EffectorLocation);
	FVector const CSEffectorLocation = CSEffectorTransform.GetLocation();

	// Update EffectorLocation2 if it is based off a bone position
	FTransform CSEffectorTransform2 = GetTargetTransform(Output.AnimInstanceProxy->GetComponentTransform(), Output.Pose, EffectorTarget2, EffectorLocationSpace2, EffectorLocation2);
	FVector const CSEffectorLocation2 = CSEffectorTransform2.GetLocation();

	TArray<FCCDIKChainLink> IKChain1 = CreateIKChain(m_RightHandBone, m_RightClavicleBone);
	TArray<FCCDIKChainLink> IKChain2 = CreateIKChain(m_LeftFootBone, m_LeftThighBone);
	TArray<float> RotationLimitArray = CreateRotationLimitArray(IKChain1.Num());
	TArray<float> RotationLimitArray2 = CreateRotationLimitArray(IKChain2.Num());
	
	int32 NumIKActions = IKChainList.Num();

	bool bBoneLocationUpdated = false;

	for (int32 iBone = 0; iBone < NumBones; iBone++)
	{
		for (int32 iChain = 0; iChain < NumIKActions; iChain++)
		{
			int32 BoneIsInChain = CheckIfJointIsRelevantToChain(IKChainList[iChain], m_CachedBoneIndicesForCurrentLOD[iBone], iChain);

			//If the bone is in the current chain, apply IK solver for that bone in that chain
			if (BoneIsInChain == 0)
			{
				//Here we call the CCDIK solver defined in CCDIK.cpp
				bBoneLocationUpdated = AnimationCore::SolveCCDIK(IKChainList[iChain], CSEffectorLocation, Precision, MaxIterations, bEnableRotationLimit, RotationLimitArray, m_CachedBoneIndicesForCurrentLOD[iBone]);
			}

			if (BoneIsInChain == 1)
			{
				//Here we call the CCDIK solver defined in CCDIK.cpp
				bBoneLocationUpdated = AnimationCore::SolveCCDIK(IKChainList[iChain], CSEffectorLocation2, Precision, MaxIterations, bEnableRotationLimit, RotationLimitArray2, m_CachedBoneIndicesForCurrentLOD[iBone]);
			}
		}
	}

	if (bBoneLocationUpdated)
	{
		for (int32 iChain = 0; iChain < NumIKActions; iChain++)
		{
			int32 NumChainLinks = IKChainList[iChain].Num();

			// First step: update bone transform positions from chain links.
			for (int32 LinkIndex = 0; LinkIndex < NumChainLinks; LinkIndex++)
			{
				FCCDIKChainLink const& ChainLink = IKChainList[iChain][LinkIndex];
				int32 CurrentBoneIndex = ChainLink.BoneIndex.GetInt();
				//Store OutBoneTransforms with new current link transform
				int32 IndexPos = GetIndexPosFromOutBoneTransforms(OutBoneTransforms, CurrentBoneIndex);
				OutBoneTransforms[IndexPos].Transform = ChainLink.Transform;

			}
		}

#if WITH_EDITOR
		DebugLines.Reset(OutBoneTransforms.Num());
		DebugLines.AddUninitialized(OutBoneTransforms.Num());
		for (int32 Index = 0; Index < OutBoneTransforms.Num(); ++Index)
		{
			DebugLines[Index] = OutBoneTransforms[Index].Transform.GetLocation();
		}
#endif // WITH_EDITOR

	}
}


bool FAnimNode_CCDIK::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	//if (EffectorLocationSpace == BCS_ParentBoneSpace || EffectorLocationSpace == BCS_BoneSpace)
	//{
	//	if (!EffectorTarget.IsValidToEvaluate(RequiredBones))
	//	{
	//		return false;
	//	}
	//}

	//// Allow evaluation if all parameters are initialized and TipBone is child of RootBone
	//return
	//	(
	//		TipBone.IsValidToEvaluate(RequiredBones)
	//		&& RootBone.IsValidToEvaluate(RequiredBones)
	//		&& Precision > 0
	//		&& RequiredBones.BoneIsChildOf(TipBone.BoneIndex, RootBone.BoneIndex)
	//		);
	return true;
}



#if WITH_EDITOR
void FAnimNode_CCDIK::ResizeRotationLimitPerJoints(int32 NewSize)
{
	if (NewSize == 0)
	{
		RotationLimitPerJoints.Reset();
	}
	else if (RotationLimitPerJoints.Num() != NewSize)
	{
		int32 StartIndex = RotationLimitPerJoints.Num();
		RotationLimitPerJoints.SetNum(NewSize);
		for (int32 Index = StartIndex; Index < RotationLimitPerJoints.Num(); ++Index)
		{
			RotationLimitPerJoints[Index] = 30.f;
		}
	}
}
#endif 

void FAnimNode_CCDIK::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	DECLARE_SCOPE_HIERARCHICAL_COUNTER_ANIMNODE(InitializeBoneReferences)
	TipBone.Initialize(RequiredBones);
	RootBone.Initialize(RequiredBones);
	EffectorTarget.InitializeBoneReferences(RequiredBones);
}

void FAnimNode_CCDIK::GatherDebugData(FNodeDebugData& DebugData)
{
	DECLARE_SCOPE_HIERARCHICAL_COUNTER_ANIMNODE(GatherDebugData)
	FString DebugLine = DebugData.GetNodeName(this);

	DebugData.AddDebugItem(DebugLine);
	ComponentPose.GatherDebugData(DebugData);
}


//FUNCTION CREATED BY ME
void FAnimNode_CCDIK::DrawLine(FVector P1_in, FVector P2_in, FColor color, float thicknessMult)
{
	static float thickness = 3.0f;

	float useThickness = thickness * thicknessMult;

	UWorld* pWorld = m_MyWorld;

	FVector P1 = P1_in;

	FVector P2 = P2_in;

	TFunction<void()> funcLambda =

		[pWorld, P1, P2, color, useThickness]() {

		DrawDebugLine(pWorld, P1, P2, color, false, -1.0f, 0, useThickness);

	};

	FFunctionGraphTask::CreateAndDispatchWhenReady(funcLambda, TStatId(), NULL, ENamedThreads::GameThread);
}

//FUNCTION CREATED BY ME
void FAnimNode_CCDIK::printsmth()
{
	if (GEngine) 
	{
		FString Stringg = FString::Printf(TEXT("World delta for current frame equals %f"), 0);
		GEngine->AddOnScreenDebugMessage(-1, 1.0f, FColor::Yellow, Stringg);
	}
}



//void FAnimNode_CCDIK::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
//{
//					// Store references to the context and out transforms
//					m_ComponentSpacePoseContext = &Output;
//					m_OutBoneTransforms = &OutBoneTransforms;
//					// If the LOD has changed, re-cache link bones
//					if (m_SkelComp->PredictedLODLevel != m_LastBoneIndicesCacheLOD)
//					{
//						const int32 NumBodies = m_SkelComp->Bodies.Num();
//						if (m_LastBoneIndicesCacheLOD == -1) 
//						{
//							m_CachedBoneIndicesForCurrentLOD.SetNum(NumBodies);
//						}
//			
//						m_LastBoneIndicesCacheLOD = m_SkelComp->PredictedLODLevel;
//						const FBoneContainer& BoneContainer = m_ComponentSpacePoseContext->Pose.GetPose().GetBoneContainer();
//						const auto& BoneIndices = BoneContainer.GetBoneIndicesArray();
//						for (int32 iLinkOnCBR = 0; iLinkOnCBR < NumBodies; iLinkOnCBR++)
//						{
//							FBodyInstance* BodyInst = m_SkelComp->Bodies[iLinkOnCBR];
//							const int32 BoneIndex = BodyInst->InstanceBoneIndex;
//							m_CachedBoneIndicesForCurrentLOD[iLinkOnCBR] = BoneIndices.Find(BoneIndex);
//						}
//					}
//
//	//The purpose of the following operations up to the invocation of the solver, is to obtain a copy of the string of bones to be worked on, 
//	//storing all its transforms in an array called Chain. Likewise, the transforms in component space are stored in another array called OutBoneTransforms.
//
//	DECLARE_SCOPE_HIERARCHICAL_COUNTER_ANIMNODE(EvaluateSkeletalControl_AnyThread)
//	const FBoneContainer& BoneContainer = Output.Pose.GetPose().GetBoneContainer();
//
//	// Update EffectorLocation if it is based off a bone position
//	FTransform CSEffectorTransform = GetTargetTransform(Output.AnimInstanceProxy->GetComponentTransform(), Output.Pose, EffectorTarget, EffectorLocationSpace, EffectorLocation);
//	FVector const CSEffectorLocation = CSEffectorTransform.GetLocation();
//
//	// Gather all bone indices between root and tip.
//	TArray<FCompactPoseBoneIndex> BoneIndices;
//
//	
//	{	//Rootbone and TipBone index declaration
//		const FCompactPoseBoneIndex RootIndex = RootBone.GetCompactPoseIndex(BoneContainer);
//		FCompactPoseBoneIndex BoneIndex = TipBone.GetCompactPoseIndex(BoneContainer);
//		//Fill the array by inserting one by one the indices of the bones in the 0 position and moving the previous ones to the right, until you reach the rootbone.
//		do
//		{
//			BoneIndices.Insert(BoneIndex, 0);
//			BoneIndex = Output.Pose.GetPose().GetParentBoneIndex(BoneIndex);
//		} while (BoneIndex != RootIndex);
//		BoneIndices.Insert(BoneIndex, 0);
//	}
//
//	//Gather transforms and initialize OutBoneTransforms array
//	int32 const NumTransforms = BoneIndices.Num();
//	OutBoneTransforms.AddUninitialized(NumTransforms);
//
//	// Gather chain links and initialize Chain array. These are non zero length bones.
//	TArray<FCCDIKChainLink> Chain;
//	Chain.Reserve(NumTransforms);
//
//	//Start filling the arrays with Root Bone
//	{
//		const FCompactPoseBoneIndex& RootBoneIndex = BoneIndices[0];
//		const FTransform& LocalTransform = Output.Pose.GetLocalSpaceTransform(RootBoneIndex);
//		const FTransform& BoneCSTransform = Output.Pose.GetComponentSpaceTransform(RootBoneIndex);
//
//		//Add Rootbone to OutBoneTransforms array and Chain array in position 0
//		OutBoneTransforms[0] = FBoneTransform(RootBoneIndex, BoneCSTransform);
//		Chain.Add(FCCDIKChainLink(BoneCSTransform, LocalTransform, 0));
//	}
//
//	// Go through remaining transforms
//	for (int32 TransformIndex = 1; TransformIndex < NumTransforms; TransformIndex++)
//	{
//		//Get the Bone Index
//		const FCompactPoseBoneIndex& BoneIndex = BoneIndices[TransformIndex];
//
//		//Get CS and Local Transforms for that BoneIndex
//		const FTransform& LocalTransform = Output.Pose.GetLocalSpaceTransform(BoneIndex);
//		const FTransform& BoneCSTransform = Output.Pose.GetComponentSpaceTransform(BoneIndex);
//		FVector const BoneCSPosition = BoneCSTransform.GetLocation();
//
//		//Fill OutBoneTransforms array
//		OutBoneTransforms[TransformIndex] = FBoneTransform(BoneIndex, BoneCSTransform);
//
//		// Calculate the combined length of this segment of skeleton
//		float const BoneLength = FVector::Dist(BoneCSPosition, OutBoneTransforms[TransformIndex - 1].Transform.GetLocation());
//
//		//If its big enough, fill Chain array
//		if (!FMath::IsNearlyZero(BoneLength))
//		{
//			Chain.Add(FCCDIKChainLink(BoneCSTransform, LocalTransform, TransformIndex));
//		}
//		else
//		{
//			// Mark this transform as a zero length child of the last link.
//			// It will inherit position and delta rotation from parent link.
//			FCCDIKChainLink & ParentLink = Chain[Chain.Num() - 1];
//			ParentLink.ChildZeroLengthTransformIndices.Add(TransformIndex);
//		}
//	}
//
//
//	//Here we call the CCDIK solver defined in CCDIK.cpp
//	bool bBoneLocationUpdated = AnimationCore::SolveCCDIK(Chain, CSEffectorLocation, Precision, MaxIterations, bEnableRotationLimit, RotationLimitPerJoints);
//
//							const FTransform& ComponentToWorld = m_SkelComp->GetComponentTransform();
//							const FTransform& ComponentToWorldInv = ComponentToWorld.Inverse();
//
//							// Convert from articulation to anim bone
//							for (int32 i = 0; i < m_InRagdollBones.Num(); i++)
//							{
//								int32 BoneIndex = m_InRagdollBones[i];
//								FName boneName = m_SkelComp->SkeletalMesh->RefSkeleton.GetBoneName(BoneIndex);
//								int32 BodyIndex = InAnimInstance->GetSkelMeshComponent()->GetPhysicsAsset()->FindBodyIndex(boneName);
//								check(BodyIndex != INDEX_NONE);
//								FTransform componentSpaceTransform = m_WorldSpaceBoneTM[BodyIndex] * ComponentToWorldInv;
//								componentSpaceTransform.DiagnosticCheck_IsValid();
//								m_OutBoneTransforms->Add(FBoneTransform(FCompactPoseBoneIndex(BoneIndex), componentSpaceTransform));
//							}
//
//	// If we moved some bones, update bone transforms.
//	if (bBoneLocationUpdated)
//	{
//		int32 NumChainLinks = Chain.Num();
//
//		// First step: update bone transform positions from chain links.
//		for (int32 LinkIndex = 0; LinkIndex < NumChainLinks; LinkIndex++)
//		{
//			FCCDIKChainLink const & ChainLink = Chain[LinkIndex];
//			//Store OutBoneTransforms with new current link transform
//			OutBoneTransforms[ChainLink.TransformIndex].Transform = ChainLink.Transform;
//
//			// If there are any zero length children, update position of those
//			int32 const NumChildren = ChainLink.ChildZeroLengthTransformIndices.Num();
//			for (int32 ChildIndex = 0; ChildIndex < NumChildren; ChildIndex++)
//			{
//				OutBoneTransforms[ChainLink.ChildZeroLengthTransformIndices[ChildIndex]].Transform = ChainLink.Transform;
//			}
//		}
//
//		//DrawLine(Chain[2].Transform.GetLocation(), Chain[3].Transform.GetLocation(), FColor::Green, 10);
//
//#if WITH_EDITOR
//		DebugLines.Reset(OutBoneTransforms.Num());
//		DebugLines.AddUninitialized(OutBoneTransforms.Num());
//		for (int32 Index = 0; Index < OutBoneTransforms.Num(); ++Index)
//		{
//			DebugLines[Index] = OutBoneTransforms[Index].Transform.GetLocation();
//		}
//#endif // WITH_EDITOR
//
//	}
//}




