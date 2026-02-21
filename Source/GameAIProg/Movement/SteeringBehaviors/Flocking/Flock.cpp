#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"


Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{pWorld}
	, FlockSize{ FlockSize }
	, pAgentToEvade{pAgentToEvade}
{
	Agents.SetNum(FlockSize);

 // TODO: initialize the flock and the memory pool
	Neighbors.SetNum(FlockSize);
	NrOfNeighbors = 0;
	
	for (int i = 0; i < FlockSize; ++i)
	{
		// simple random-ish spread so they don't start on the same spot
		const float x = FMath::FRandRange(-WorldSize * 0.3f, WorldSize * 0.3f);
		const float y = FMath::FRandRange(-WorldSize * 0.3f, WorldSize * 0.3f);

		Agents[i] = pWorld->SpawnActor<ASteeringAgent>(
			AgentClass,
			FVector{ x, y, 90.f },
			FRotator::ZeroRotator
		);
	}

	//Create a cohesion behavior that can access THIS flock
	auto* pCohesion = new Cohesion(this);
	auto* pSeparation = new Separation(this);
	//Assign cohesion to all agents (test)
	for (ASteeringAgent* a : Agents)
	{
		if (IsValid(a))
			a->SetSteeringBehavior(pSeparation);
	}
}

Flock::~Flock()
{
 // TODO: Cleanup any additional data
}

void Flock::Tick(float DeltaTime)
{
	for (auto pAgent : Agents)
	{
		if (!IsValid(pAgent))
		{
			continue;
		}
		
		RegisterNeighbors(pAgent);
		//Later: SteeringBehaviours will read Neighbours[0 to NrOfNeigbhours-1] 
		pAgent->Tick(DeltaTime);
	}
	
 // TODO: update the flock
 // TODO: for every agent:
  // TODO: register the neighbors for this agent (-> fill the memory pool with the neighbors for the currently evaluated agent)
  // TODO: update the agent (-> the steeringbehaviors use the neighbors in the memory pool)
  // TODO: trim the agent to the world
}

void Flock::RenderDebug()
{
 // TODO: Render all the agents in the flock
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();

  // TODO: implement ImGUI checkboxes for debug rendering here

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

  // TODO: implement ImGUI sliders for steering behavior weights here
		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
 // TODO: Debugrender the neighbors for the first agent in the flock
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	// TODO: Implement
	NrOfNeighbors = 0;
	
	//Squared radius for cheaper distance check later
	const float radiusSq = NeighborhoodRadius * NeighborhoodRadius;
	const FVector2D agentPos = pAgent->GetPosition();
	
	//Check for all flock agents
	for (ASteeringAgent* const pOther : Agents)
	{
		// skip invalid and skip self
		if (!IsValid(pOther) || pOther == pAgent)
			continue;

		const FVector2D toOther = pOther->GetPosition() - agentPos;
		const float distSq = toOther.SizeSquared();

		if (distSq <= radiusSq)
		{
			if (NrOfNeighbors < Neighbors.Num())
			{
				Neighbors[NrOfNeighbors] = pOther;
				NrOfNeighbors++;
			}
		}
	}
	
}
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	//if no neighbour, no need to calculate the rest
	FVector2D avgPosition = FVector2D::ZeroVector;
	if (NrOfNeighbors == 0) return avgPosition;
	
	//Sum all neigbour positions
	for (int agentNumber{0}; agentNumber<NrOfNeighbors;agentNumber++)
	{
		avgPosition += Neighbors[agentNumber]->GetPosition();
	}
	//Devide by count
	avgPosition /= NrOfNeighbors;
	return avgPosition;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D avgVelocity = FVector2D::ZeroVector;
	if (NrOfNeighbors == 0) return avgVelocity;
	for (int agentNumber{0}; agentNumber<NrOfNeighbors; agentNumber++ )
	{
		avgVelocity = avgVelocity + Neighbors[agentNumber]->GetLinearVelocity();
	}
	avgVelocity /= NrOfNeighbors;
	return avgVelocity;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
 // TODO: Implement
}

