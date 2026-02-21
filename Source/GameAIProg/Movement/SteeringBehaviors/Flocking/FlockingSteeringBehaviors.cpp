#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput Steering{};
	
	if (pFlock == nullptr) return Steering;
	
	if (pFlock->GetNrOfNeighbors() == 0)
	{
		Steering.LinearVelocity = FVector2D::ZeroVector;
		return Steering;
	}
	
	const FVector2D avgPosition = pFlock->GetAverageNeighborPos();
	FTargetData cohesionTarget{};
	cohesionTarget.Position = avgPosition;
	
	SetTarget(cohesionTarget);
	return Seek::CalculateSteering(deltaT, pAgent);
}

//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput Steering{};
	if (pFlock == nullptr) return Steering;
	const int cout = pFlock->GetNrOfNeighbors();
	if (cout == 0)
	{
		Steering.LinearVelocity = FVector2D::ZeroVector;
		return Steering;
	}
	
	const TArray<ASteeringAgent*>& neightbours = pFlock->GetNeighbors();
	const FVector2D agentPos = pAgent.GetPosition();
	
	FVector2D force = FVector2D::ZeroVector;
	
	//push away for eadch neighbour
	for (int i = 0; i < cout; i++)
	{
		ASteeringAgent* neightbour = neightbours[i];
		if (!IsValid(neightbour)) continue;
		const FVector2D away = agentPos - neightbour->GetPosition();
		const float distance = away.Size();
		//(away / dist) = normalized away direction
		// (1 / dist)    = strength
		force += (away / distance) * (1.f / distance);
	}
	
	const float maxSpeed = pAgent.GetMaxLinearSpeed();
	if (!force.IsNearlyZero())
	{
		// scale up to speed space and clamp
		force *= maxSpeed;
		force = force.GetClampedToMaxSize(maxSpeed);
	}

	Steering.LinearVelocity = force;
	return Steering;
}


//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput Steering{};
	if (pFlock == nullptr) return Steering;
	
	if (pFlock->GetNrOfNeighbors() == 0)
	{
		Steering.LinearVelocity = FVector2D::ZeroVector;
		return Steering;
	}
	
	FVector2D avgVelocity = pFlock->GetAverageNeighborVelocity();
	
	avgVelocity = avgVelocity.GetClampedToMaxSize(pAgent.GetMaxLinearSpeed());
	Steering.LinearVelocity = avgVelocity;
	return Steering;
}