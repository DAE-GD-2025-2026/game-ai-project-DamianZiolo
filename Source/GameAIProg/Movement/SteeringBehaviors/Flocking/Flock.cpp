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
	
	pCohesionBehavior = std::make_unique<Cohesion>(this);
	pSeparationBehavior = std::make_unique<Separation>(this);
	pVelMatchBehavior = std::make_unique<VelocityMatch>(this);
	pSeekBehavior = std::make_unique<Seek>();
	pWanderBehavior = std::make_unique<Wander>();
	pBlendedSteering = std::make_unique<BlendedSteering>(std::vector<BlendedSteering::WeightedBehavior>{
	{ pSeparationBehavior.get(),  1.5f },  
	{ pCohesionBehavior.get(),    0.6f },  
	{ pVelMatchBehavior.get(),    0.8f },  
	{ pWanderBehavior.get(),      0.4f },  

});
	pAgentToEvade->SetSteeringBehavior(pSeekBehavior.get());
	

	if (pAgentToEvade)
	{
		pEvadeBehavior = std::make_unique<Evade>();

		 //evade first, then blended flocking
		pPrioritySteering = std::make_unique<PrioritySteering>(
			std::vector<ISteeringBehavior*>{ pEvadeBehavior.get(), pBlendedSteering.get() }
		);
	}
	
	for (int i = 0; i < FlockSize; ++i)
	{
		// simple random-ish spread so they don't start on the same spot
		const float x = FMath::FRandRange(-WorldSize * 0.3f, WorldSize * 0.3f);
		const float y = FMath::FRandRange(-WorldSize * 0.3f, WorldSize * 0.3f);
		FActorSpawnParameters params{};
		params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		Agents[i] = pWorld->SpawnActor<ASteeringAgent>(
			AgentClass,
			FVector{ x, y, 90.f },
			FRotator::ZeroRotator,
			params
		);
		Agents[i]->SetActorTickEnabled(false);
	}

	//Create a cohesion behavior that can access THIS flock

	//Assign cohesion to all agents (test)
	for (ASteeringAgent* a : Agents)
	{
		if (!IsValid(a))
		{
			continue;
		}
			if (pPrioritySteering)
				a->SetSteeringBehavior(pPrioritySteering.get());
			else
				a->SetSteeringBehavior(pBlendedSteering.get());
	}
	
	
	
}

Flock::~Flock()
{
 
	// TODO: Cleanup any additional data
}

void Flock::Tick(float DeltaTime)
{
	if (pEvadeBehavior && IsValid(pAgentToEvade))
	{
		FTargetData targetToEvade{};
		targetToEvade.Position        = pAgentToEvade->GetPosition();
		targetToEvade.Orientation     = pAgentToEvade->GetRotation();
		targetToEvade.LinearVelocity  = pAgentToEvade->GetLinearVelocity();
		targetToEvade.AngularVelocity = pAgentToEvade->GetAngularVelocity();
		pEvadeBehavior->SetTarget(targetToEvade);
	}
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
		if (pBlendedSteering)
		{
			// get pointers to weights
			float* wSep  = pBlendedSteering->GetWeight(pSeparationBehavior.get());
			float* wCoh  = pBlendedSteering->GetWeight(pCohesionBehavior.get());
			float* wAli  = pBlendedSteering->GetWeight(pVelMatchBehavior.get());
			float* wWan  = pBlendedSteering->GetWeight(pWanderBehavior.get());

			// local copies
			if (wSep)
			{
				float tmp = *wSep;
				if (ImGui::SliderFloat("Separation", &tmp, 0.f, 3.f, "%.2f"))
					*wSep = tmp;
			}
			if (wCoh)
			{
				float tmp = *wCoh;
				if (ImGui::SliderFloat("Cohesion", &tmp, 0.f, 3.f, "%.2f"))
					*wCoh = tmp;
			}
			if (wAli)
			{
				float tmp = *wAli;
				if (ImGui::SliderFloat("Alignment", &tmp, 0.f, 3.f, "%.2f"))
					*wAli = tmp;
			}
			if (wWan)
			{
				float tmp = *wWan;
				if (ImGui::SliderFloat("Wander", &tmp, 0.f, 3.f, "%.2f"))
					*wWan = tmp;
			}
		}
		
		
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
 pSeekBehavior->SetTarget(Target);
}

