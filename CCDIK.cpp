// Copyright Epic Games, Inc. All Rights Reserved.

#include "CCDIK.h"

namespace AnimationCore
{
#pragma optimize("", off)

	bool SolveCCDIK(TArray<FCCDIKChainLink>& ChainOfLinks, const FVector& TargetPosition, float Tolerance, float MaxNumIteration, bool bEnableRotationLimit, const TArray<float>& RotationLimitPerJoints, int32 CurrentBoneIndex)
	{
		//Obtain number of links in the chain
		int32 const NumChainLinks = ChainOfLinks.Num();
		//Set the TipBone as the last link
		FCCDIKChainLink& TipBone = ChainOfLinks[NumChainLinks - 1];
		//Calculate distance from TipBone to target
		float TipToTargetDistance = FVector::Dist(TipBone.Transform.GetLocation(), TargetPosition);

						int32 BoneInChainIdex = -1;
						for (int32 LinkIndex = 0; LinkIndex < NumChainLinks; LinkIndex++)
						{
							if (CurrentBoneIndex == ChainOfLinks[LinkIndex].BoneIndex.GetInt())
							{
								BoneInChainIdex = LinkIndex;

								if (BoneInChainIdex == NumChainLinks - 1)
								{
									return true;
								}
							}
							
						}
						if (BoneInChainIdex == -1)
						{
							return false;
						}

		if (TipToTargetDistance > Tolerance)
		{
			FCCDIKChainLink& CurrentLink = ChainOfLinks[BoneInChainIdex];
			//Vector from Link to TipBone
			FVector CurrentLinkToTip = TipBone.Transform.GetLocation() - CurrentLink.Transform.GetLocation();
			//Vector from Link to Target
			FVector CurrentLinkToTarget = TargetPosition - CurrentLink.Transform.GetLocation();
			//Normalization of the two vectors
			FVector CurrentLinkToTipNorm = CurrentLinkToTip.GetUnsafeNormal();
			FVector CurrentLinkToTargetNorm = CurrentLinkToTarget.GetUnsafeNormal();
							
			float RotationLimitPerJointInRadian = FMath::DegreesToRadians(RotationLimitPerJoints[BoneInChainIdex]);
			//Angle that needs to be rotated
			float Angle = FMath::ClampAngle(FMath::Acos(FVector::DotProduct(CurrentLinkToTipNorm, CurrentLinkToTargetNorm)), -RotationLimitPerJointInRadian, RotationLimitPerJointInRadian);

			//Check if Link can rotate
			bool bCanRotate = (FMath::Abs(Angle) > KINDA_SMALL_NUMBER) && (!bEnableRotationLimit || RotationLimitPerJointInRadian > CurrentLink.CurrentAngleDelta);
			if (bCanRotate)
			{
				//If Angle is too big and we go out of the limits
				if (bEnableRotationLimit) 
				
				{
					if (RotationLimitPerJointInRadian < CurrentLink.CurrentAngleDelta + Angle)
					{
						Angle = RotationLimitPerJointInRadian - CurrentLink.CurrentAngleDelta;
					}
					//Update CurrentAngle of the CurrentLink
					CurrentLink.CurrentAngleDelta += Angle;

				}
					

				//Once we have the angle, we need the rotation axis
				FVector RotationAxis = FVector::CrossProduct(CurrentLinkToTipNorm, CurrentLinkToTargetNorm);

				if (RotationAxis.SizeSquared() > 0.0f)
				{
					//Normalize RotationAxis
					FVector RotationAxisNorm = RotationAxis.GetUnsafeNormal();
					//Convert Angle and Rotation axis to Quaternions
					FQuat AngleRotation(RotationAxis, Angle);
					//Combine AngleRotation with existing Rotation into a NewRotation
					FQuat NewRotation = AngleRotation * CurrentLink.Transform.GetRotation();
					FQuat NewRotationNorm = NewRotation.GetNormalized();
					//Set current rotation to new rotation
					CurrentLink.Transform.SetRotation(NewRotationNorm);
						
					//if the current link has a parent, we need to update our local transform
					if (BoneInChainIdex > 0)
					{
						FCCDIKChainLink& CurrentParent = ChainOfLinks[BoneInChainIdex - 1];
						CurrentLink.LocalTransform = CurrentLink.Transform.GetRelativeTransform(CurrentParent.Transform);
						CurrentLink.LocalTransform.NormalizeRotation();
					}

					//if the current link has children, update the component transform of all of them

					if (BoneInChainIdex < NumChainLinks - 1)
					{
						FTransform CurrentParentTransform = CurrentLink.Transform;
						for (int32 ChildLinkIndex = BoneInChainIdex + 1; ChildLinkIndex < NumChainLinks; ChildLinkIndex++)
						{
							FCCDIKChainLink& ChildLink = ChainOfLinks[ChildLinkIndex];
							ChildLink.Transform = ChildLink.LocalTransform * CurrentParentTransform;
							ChildLink.Transform.NormalizeRotation();
							CurrentParentTransform = ChildLink.Transform;
						}
					}

					return true;

				}

			}
		}


		return false;
	}
#pragma optimize("", on)
}

