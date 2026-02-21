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

//*************************
//VELOCITY MATCH (FLOCKING)
