#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	//Todo show a cool thing :D 

	//Add debug rendering for grades :)

	return Steering;
}
