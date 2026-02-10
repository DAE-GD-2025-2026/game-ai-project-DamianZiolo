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

    //if originalSpeed is lower than 0 it means it's not set up
    if (originalMaxSpeed < 0.f)
    {
        originalMaxSpeed = Agent.GetMaxLinearSpeed();
    }


	const FVector2D toTarget = Target.Position - Agent.GetPosition();
    const float distance = toTarget.Size(); 
    Steering.LinearVelocity = toTarget;

    if (distance <= targetRadius)
    {
        Agent.SetMaxLinearSpeed(0);
    }
    else if (distance <= slowRadius)
    {
        // scale speed down as we get closer to target
        float scale = (distance - targetRadius) / (slowRadius - targetRadius);
        scale = FMath::Clamp(scale, 0.f, 1.f);

        Agent.SetMaxLinearSpeed(scale * originalMaxSpeed );
    }
    else
    {
        Agent.SetMaxLinearSpeed(originalMaxSpeed);
    }


	return Steering;
}

SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};

    FVector2D toTarget = Target.Position - Agent.GetPosition();
    if (toTarget.IsNearlyZero())
    {
        Steering.AngularVelocity = 0.f;
        return Steering;
    }

    float desiredOrientation = FMath::Atan2(toTarget.Y, toTarget.X);
    float currentYawRad = FMath::DegreesToRadians(Agent.GetActorRotation().Yaw);
    //This is smarter way becuase instead of rotating us 358 degrees, it's going to rotate us 2 degrees in other direction!
    float rotation = FMath::FindDeltaAngleRadians(currentYawRad, desiredOrientation);

    const float targetAngle = FMath::DegreesToRadians(1.f); //if difference is smaler than 1 degrees, then don't move
    if (FMath::Abs(rotation) < targetAngle)
    {
        Steering.AngularVelocity = 0.f;
        return Steering;
    }

    Steering.AngularVelocity = rotation / timeToTarget;
    Steering.AngularVelocity = FMath::Clamp(
    Steering.AngularVelocity,
    -Agent.GetMaxAngularSpeed(),
    Agent.GetMaxAngularSpeed()
    );

    

    return Steering;
}

SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};
   
    FVector2D pursuerPosition{ Agent.GetPosition() };
    float pursuerSpeed{ Agent.GetMaxLinearSpeed() };
    FVector2D targetPos{ Target.Position };
    FVector2D targetVel{ Target.LinearVelocity };

    FVector2D toTarget{ targetPos - pursuerPosition };
    float distance = toTarget.Size();
    float timeToReach = 0;
    if (pursuerSpeed != 0)
    {
        timeToReach = distance / pursuerSpeed;
    }
    
    FVector2D predictedOffset = targetVel * timeToReach;
    FVector2D predictedPos = targetPos + predictedOffset;

    Steering.LinearVelocity = predictedPos - Agent.GetPosition();
  

    return Steering;
}
