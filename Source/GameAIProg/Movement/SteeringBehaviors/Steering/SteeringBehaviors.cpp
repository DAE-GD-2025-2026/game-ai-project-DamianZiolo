#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "DrawDebugHelpers.h"


//SEEK
//*******
// TODO: Do the Week01 assignment :^)

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};

    const FVector2D toTarget = Target.Position - Agent.GetPosition();
    const float distance = toTarget.Size();

    const float stopRadius = 1.f;
    if (distance <= stopRadius)
    {
        Steering.LinearVelocity = FVector2D::ZeroVector;
        return Steering;
    }

    Steering.LinearVelocity = toTarget;

    //Debug part
    const FVector start = FVector(Agent.GetPosition(), 0.f);
    const FVector end = FVector(Target.Position, 0.f);

    // DrawDebugLine(
    //     Agent.GetWorld(),
    //     start,
    //     end,
    //     FColor::Green,
    //     false,   // persistent lines
    //     0.f,     // life time (0 = one frame)
    //     0,
    //     2.f      // thickness
    // );



    return Steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

    const FVector2D awayFromTarget = Agent.GetPosition() - Target.Position;
    Steering.LinearVelocity = awayFromTarget;

    // Debug line: direction of flee
    const FVector start = FVector(Agent.GetPosition(), 0.f);
    const FVector2D dir = awayFromTarget.GetSafeNormal();
    const FVector end = start + FVector(dir, 0.f) * 200.f;

    DrawDebugLine(
        Agent.GetWorld(),
        start,
        end,
        FColor::Red,
        false,
        0.f,
        0,
        2.f
    );

	return Steering;
}

void Arrive::SetTargetRadius(float radius)
{
    m_Radius = radius;
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


    const FVector targetWorld = FVector(Target.Position, 0.f);

    // Target destination
    DrawDebugSphere(
        Agent.GetWorld(),
        targetWorld,
        15.f,          // size of the ball
        16,
        FColor::Red,
        false,
        0.f
    );

    // Slow Radius
    DrawDebugCircle(
        Agent.GetWorld(),
        targetWorld,
        slowRadius,
        64,
        FColor::Yellow,
        false,
        0.f,
        0,
        2.f,
        FVector(1, 0, 0),   // X axis
        FVector(0, 1, 0),   // Y axis
        false
    );

    // Target Radius
    DrawDebugCircle(
        Agent.GetWorld(),
        targetWorld,
        targetRadius,
        64,
        FColor::Blue,
        false,
        0.f,
        0,
        2.f,
        FVector(1, 0, 0),
        FVector(0, 1, 0),
        false
    );


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

    const FVector start = FVector(Agent.GetPosition(), 0.f);

    // Goal direction rotation:
    const FVector2D desiredDir(FMath::Cos(desiredOrientation), FMath::Sin(desiredOrientation));
    const FVector end = start + FVector(desiredDir, 0.f) * 200.f;

    DrawDebugLine(
        Agent.GetWorld(),
        start,
        end,
        FColor::Cyan,
        false,
        0.f,
        0,
        2.f
    );

    //curent Direction rotation:
    const FVector2D currentDir(FMath::Cos(currentYawRad), FMath::Sin(currentYawRad));

    // DrawDebugLine(
    //     Agent.GetWorld(),
    //     start,
    //     start + FVector(currentDir, 0.f) * 200.f,
    //     FColor::Green,
    //     false,
    //     0.f,
    //     0,
    //     2.f
    // );


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
  
    // // Agent->predicted position line
    //     DrawDebugLine(
    //         Agent.GetWorld(),
    //         FVector(Agent.GetPosition(), 0.f),
    //         FVector(predictedPos, 0.f),
    //         FColor::Green,
    //         false,
    //         0.f,
    //         0,
    //         2.f
    //     );

    // Predicted position marker
    DrawDebugSphere(
        Agent.GetWorld(),
        FVector(predictedPos, 0.f),
        15.f,
        12,
        FColor::Yellow,
        false,
        0.f
    );
    return Steering;
}

SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};
    const float EvadeRadius{500.f};
    FVector2D pursuerPosition{ Agent.GetPosition() };
    float pursuerSpeed{ Agent.GetMaxLinearSpeed() };
    FVector2D targetPos{ Target.Position };
    FVector2D targetVel{ Target.LinearVelocity };

    FVector2D toTarget{ targetPos - pursuerPosition };
    float distance = toTarget.Size();
    
    if (distance > EvadeRadius)
    {
        Steering.IsValid = false;
        Steering.LinearVelocity = FVector2D::ZeroVector;
        Steering.AngularVelocity = 0.f;
        return Steering;
    }
    Steering.IsValid = true;
    
    float timeToReach = 0;
    if (pursuerSpeed != 0)
    {
        timeToReach = distance / pursuerSpeed;
    }

    FVector2D predictedOffset = targetVel * timeToReach;
    FVector2D predictedPos = targetPos + predictedOffset;

    Steering.LinearVelocity = Agent.GetPosition() - predictedPos;

    // Predicted position marker where the pursuer is expected to be
    DrawDebugSphere(
        Agent.GetWorld(),
        FVector(predictedPos, 0.f),
        15.f,
        12,
        FColor::Yellow,
        false,
        0.f
    );

    // Line from agent to predicted position
    DrawDebugLine(
        Agent.GetWorld(),
        FVector(Agent.GetPosition(), 0.f),
        FVector(predictedPos, 0.f),
        FColor::Red,
        false,
        0.f,
        0,
        2.f
    );

    // Evade direction line
    const FVector2D fleeDir = (Agent.GetPosition() - predictedPos).GetSafeNormal();
    DrawDebugLine(
        Agent.GetWorld(),
        FVector(Agent.GetPosition(), 0.f),
        FVector(Agent.GetPosition(), 0.f) + FVector(fleeDir, 0.f) * 200.f,
        FColor::Green,
        false,
        0.f,
        0,
        2.f
    );


    return Steering;
}

SteeringOutput Wander::CalculateSteering(float deltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};
    //Agent forward vector
    float yawRad = FMath::DegreesToRadians(Agent.GetActorRotation().Yaw);
    FVector2D forward = FVector2D{ cos(yawRad),sin(yawRad) };
    //Circle center in front of the agent
    FVector2D circleCenter = Agent.GetPosition() + forward * m_OffsetDistance;
    //Smooth random angle change
    const float deltaAngle = FMath::FRandRange(-m_MaxAngleChance, m_MaxAngleChance);
    m_WanderAngle += deltaAngle;

    //keeping angle bounded to avoid huge value and crashing code after some time
    if (m_WanderAngle > PI)  m_WanderAngle -= 2.f * PI;
    if (m_WanderAngle < -PI) m_WanderAngle += 2.f * PI;

    //Point on the circle
    const FVector2D offsetOnCircle(FMath::Cos(m_WanderAngle), FMath::Sin(m_WanderAngle));
    const FVector2D wanderTarget = circleCenter + offsetOnCircle * m_Radius;

    //Seek towards that target
    const FVector2D toTarget = wanderTarget - Agent.GetPosition();
    Steering.LinearVelocity = toTarget;

   /* const float Z = Agent.GetActorLocation().Z;

    DrawDebugSphere(
        Agent.GetWorld(),
        FVector(wanderTarget.X, wanderTarget.Y, Z),
        12.f, 12, FColor::Red,
        false, 1.f
    );*/




    return Steering;
}

SteeringOutput WolfPack::CalculateSteering(float deltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};

    // base
    const FVector2D agentPos = Agent.GetPosition();
    const FVector2D toTarget = Target.Position - agentPos;
    const float distanceToTarget = toTarget.Size();

    // pack
    const int32 packCount = PackMates.Num();
    if (packCount <= 0)
    {
        Steering.LinearVelocity = toTarget.GetSafeNormal(); // fallback
        return Steering;
    }

    const int32 myIndex = PackMates.IndexOfByKey(&Agent);
    if (myIndex == INDEX_NONE)
    {
        Steering.LinearVelocity = toTarget.GetSafeNormal(); // fallback
        return Steering;
    }

    FVector2D desiredVelocity = FVector2D::ZeroVector;

    // chase
    if (distanceToTarget > SurroundingRange)
    {
        desiredVelocity = toTarget.GetSafeNormal();
    }
    else
    {
        // slot
        const float angleStep = 2.f * PI / static_cast<float>(packCount);
        const float myAngle = angleStep * static_cast<float>(myIndex);

        const FVector2D slotDir(
            FMath::Cos(myAngle),
            FMath::Sin(myAngle)
        );

        const FVector2D desiredPos =
            Target.Position + slotDir * SurroundRadius;

        desiredVelocity = (desiredPos - agentPos).GetSafeNormal();
    }

    // separation
    FVector2D separationForce = FVector2D::ZeroVector;

    for (ASteeringAgent* Wolf : PackMates)
    {
        if (!Wolf || Wolf == &Agent)
            continue;

        const FVector2D away = agentPos - Wolf->GetPosition();
        const float dist = away.Size();

        if (dist > 0.f && dist < SeparationRadius)
        {
            const float strength = 1.f - dist / SeparationRadius;
            separationForce += away.GetSafeNormal() * strength;
        }
    }

    // avoid
    FVector2D avoidTarget = FVector2D::ZeroVector;

    if (distanceToTarget < SurroundRadius)
    {
        const FVector2D away = agentPos - Target.Position;
        const float strength = 1.f - distanceToTarget / SurroundRadius;
        avoidTarget = away.GetSafeNormal() * strength;
    }

    // combine
    Steering.LinearVelocity =
        desiredVelocity +
        separationForce * SeparationWeight +
        avoidTarget * AvoidTargetWeight;

    return Steering;
}