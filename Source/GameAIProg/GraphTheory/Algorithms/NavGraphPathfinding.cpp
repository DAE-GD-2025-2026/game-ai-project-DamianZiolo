#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
	NavGraph* const pNavGraph, std::vector<FVector2D>& debugNodePositions, std::vector<NavLine>& debugPortals) 
{
	//Create the path to return
	std::vector<FVector2D> finalPath{};

	//Get the start and endTriangle
	auto navMeshPolygon = pNavGraph->GetNavPolygon();
	
	auto startTriangle = navMeshPolygon->GetTriangleAtPosition(startPos,true);
	auto endTriangle = navMeshPolygon->GetTriangleAtPosition(endPos,true);
	//We have valid start/end triangles and they are not the same
	if (!startTriangle||!endTriangle)
	{
		return finalPath;
	}
	
	if (*startTriangle == *endTriangle)
	{
		finalPath.emplace_back(startPos);
		finalPath.emplace_back(endPos);
		return finalPath;
	}
	//=> Start looking for a path
	//Copy the graph
	auto copiedGraph = pNavGraph->Clone();

	//Create Extra node for the Start Node (Agent's position
	auto startNode = std::make_unique<NavGraphNode>(startPos, -1);
	int startNodeId = copiedGraph->AddNode(std::move(startNode));

	auto edges = startTriangle->GetEdges();
	
	for (const auto& edge : edges)
	{
		auto edgeIndex = navMeshPolygon->FindEdgeIndex(edge);
		if (!edgeIndex.has_value())
		continue;
		
		int nodeId = copiedGraph->GetNodeIdFromEdgeIndex(edgeIndex.value());
		
		if (nodeId != Graphs::InvalidNodeId)
		{
			FVector2D nodePos = copiedGraph->GetNode(nodeId)->GetPosition();

			copiedGraph->AddConnection(startNodeId, nodeId);
			copiedGraph->AddConnection(nodeId, startNodeId);
			
		}
		
	}
	
	//Create extra node for the endNode
	auto endNode = std::make_unique<NavGraphNode>(endPos, -1);
	int endNodeId = copiedGraph->AddNode(std::move(endNode));

	auto endEdges = endTriangle->GetEdges();
	for (const auto& edge : endEdges)
	{
		auto edgeIndex = navMeshPolygon->FindEdgeIndex(edge);
		if (!edgeIndex.has_value())
		{
			continue;
		}
		
		int nodeId = copiedGraph->GetNodeIdFromEdgeIndex(edgeIndex.value());
		if (nodeId != Graphs::InvalidNodeId)
		{
			FVector2D nodePos = copiedGraph->GetNode(nodeId)->GetPosition();
			copiedGraph->AddConnection(endNodeId, nodeId);
			copiedGraph->AddConnection(nodeId, endNodeId);
		}
		
	}
	copiedGraph->SetConnectionCostsToDistances();
	
	//Run A star on new graph
	AStar pathfinder(copiedGraph.get(), HeuristicFunctions::Euclidean);
	auto nodePath = pathfinder.FindPath(
		copiedGraph->GetNode(startNodeId).get(),
		copiedGraph->GetNode(endNodeId).get() );
	
	//Debug Visualisation
	for (auto node : nodePath)
	{
		FVector2D pos = node->GetPosition();
		finalPath.emplace_back(pos);
		debugNodePositions.emplace_back(pos);
	}
	// Extra: Run optimiser on new graph (First check if everything works without SSFA!)
	debugPortals = SSFA::FindPortals(nodePath, *pNavGraph->GetNavPolygon());
	finalPath = SSFA::OptimizePortals(debugPortals, *pNavGraph->GetNavPolygon());
	
	return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}