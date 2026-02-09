#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();


	//Add debug rendering for grades :)

	return Steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	Steering.LinearVelocity = Agent.GetPosition() - Target.Position;
	return Steering;
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	const FVector2D toTarget = Target.Position - Agent.GetPosition();
    const float distance = toTarget.Size(); // or Size2D() if you want planar distance
    Steering.LinearVelocity = toTarget;

    if (distance <= targetRadius)
    {
        Agent.SetMaxLinearSpeed(0);
        // Steering.LinearVelocity = FVector2D::ZeroVector;
    }
    else if (distance <= slowRadius)
    {
        // scale speed down as we get closer to target
        float scale = (distance - targetRadius) / (slowRadius - targetRadius);
        scale = FMath::Clamp(scale, 0.f, 1.f);

        Agent.SetMaxLinearSpeed(scale * 1000.f );
    }
    else
    {
        Agent.SetMaxLinearSpeed(1000.f);
    }


	return Steering;
}
