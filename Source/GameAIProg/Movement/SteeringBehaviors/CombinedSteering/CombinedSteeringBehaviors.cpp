
#include "CombinedSteeringBehaviors.h"
#include <algorithm>
#include "../SteeringAgent.h"

BlendedSteering::BlendedSteering(const std::vector<WeightedBehavior>& WeightedBehaviors)
	:WeightedBehaviors(WeightedBehaviors)
{};

//****************
//BLENDED STEERING
SteeringOutput BlendedSteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	//Result
	SteeringOutput result = {};
	// TODO: Calculate the weighted average steeringbehavior
	
	FVector2D sumLinear = FVector2D::ZeroVector;
	float sumAngular = 0.f;
	float sumWeights = 0.f;

	//for each behaviour, calculate steering and add it do weight
	for (const WeightedBehavior& behavior : WeightedBehaviors)
	{
		//skip empty/invalid behaviours
		if (!behavior.pBehavior || behavior.Weight <= 0.f)
		{
			continue;
		}
		//Calculate steering of THIS behaviour(Seek, Wander, etc)
		const SteeringOutput steering = behavior.pBehavior->CalculateSteering(DeltaT, Agent);
		
		//We're adding steering to sum
		const FVector2D dir = steering.LinearVelocity.GetSafeNormal(); //we need to take normal becuse seek vector is biger than wander and that's why seek is ,,stronger"
		sumLinear += dir * behavior.Weight;
		sumAngular += steering.AngularVelocity * behavior.Weight;
		
		//We sum weights
		sumWeights += behavior.Weight;
	}
	
	
	//if we have any weights, we divide by it
	if (sumWeights > 0.f)
	{
		const FVector2D resultDir = sumLinear / sumWeights;
		result.LinearVelocity = resultDir.GetSafeNormal() * Agent.GetMaxLinearSpeed();
		result.AngularVelocity = sumAngular / sumWeights;
	}
	// TODO: Add debug drawing
		const FVector start = FVector(Agent.GetPosition(), 0.f);
		const FVector2D dir2D = result.LinearVelocity.GetSafeNormal();
		const FVector end = start + FVector(dir2D, 0.f) * 200.f;

		DrawDebugLine(Agent.GetWorld(), start, end, FColor::Magenta, false, 0.f, 0, 2.f);

	
	return result;
}

float* BlendedSteering::GetWeight(ISteeringBehavior* const SteeringBehavior)
{
	auto it = find_if(WeightedBehaviors.begin(),
		WeightedBehaviors.end(),
		[SteeringBehavior](const WeightedBehavior& Elem)
		{
			return Elem.pBehavior == SteeringBehavior;
		}
	);

	if(it!= WeightedBehaviors.end())
		return &it->Weight;
	
	return nullptr;
}

//*****************
//PRIORITY STEERING
SteeringOutput PrioritySteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering = {};

	for (ISteeringBehavior* const pBehavior : m_PriorityBehaviors)
	{
		Steering = pBehavior->CalculateSteering(DeltaT, Agent);

		if (Steering.IsValid)
			break;
	}

	//If non of the behavior return a valid output, last behavior is returned
	return Steering;
}