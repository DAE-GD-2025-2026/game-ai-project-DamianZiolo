#include "NavGraph.h"

#include "NavGraphNode.h"

GameAI::NavGraph::NavGraph(std::unique_ptr<TriPolygon> && NavPoly)
	: Graph{false}
	, pNavPoly{std::move(NavPoly)}
{
	CreateNavigationGraph();
}

GameAI::NavGraph::NavGraph(const NavGraph& Other)
	: Graph(false)
{
	Nodes.reserve(Other.Nodes.size());
	for (std::unique_ptr<Node> const & OtherNode : Other.Nodes)
	{
		Nodes.push_back(std::make_unique<NavGraphNode>(*dynamic_cast<NavGraphNode*>(OtherNode.get())));
	}
        
	Connections.reserve(Other.Connections.size());
	for (std::unique_ptr<Connection> const & OtherConnection : Other.Connections)
	{
		Connections.push_back(std::make_unique<Connection>(*OtherConnection.get()));
	}
}

std::unique_ptr<GameAI::NavGraph> GameAI::NavGraph::Clone() const
{
	return std::make_unique<NavGraph>(*this);
}

int GameAI::NavGraph::GetNodeIdFromEdgeIndex(int EdgeIdx) const
{
	if (EdgeIdx >= 0)
	{
		for (auto const & pNode : Nodes)
		{
			if (reinterpret_cast<NavGraphNode*>(pNode.get())->GetEdgeIdx() == EdgeIdx)
			{
				return pNode->GetId();
			}
		}
	}
	
	return Graphs::InvalidNodeId;
}

void GameAI::NavGraph::CreateNavigationGraph()
{
	//1. Go over all the edges of the navigation mesh and create nodes
	auto const& edges = pNavPoly->GetEdges();
	auto const& triangles = pNavPoly->GetTriangles();
	for (int EdgeIndex = 0; EdgeIndex < edges.size(); ++EdgeIndex)
	{
		const auto& currentEdge = edges[EdgeIndex];
		int triangleCount = 0;
		for (auto const & triangle : triangles)
		{
			if (triangle.HasEdge(currentEdge))
			{
				triangleCount++;
			}
		}
		
		if (triangleCount == 2)
		{
			// Create node here
			FVector p1 = currentEdge.GetP1(*pNavPoly);
			FVector p2 = currentEdge.GetP2(*pNavPoly);
			FVector pointMiddle = (p1 + p2) / 2;
			
			auto newNode = std::make_unique<NavGraphNode>(FVector2D(pointMiddle),EdgeIndex);
			AddNode(std::move(newNode) );
		}
		
	}
	
	for (auto const & triangle : triangles)
	{
		auto triangleEdges = triangle.GetEdges();
		std::vector<int> nodeIndexs;
		for (auto const & edge : triangleEdges)
		{
			auto edgeIdex = pNavPoly->FindEdgeIndex(edge);
			if (!edgeIdex.has_value())
			{
				continue;
			}
			
			int nodeId = GetNodeIdFromEdgeIndex(edgeIdex.value());
			if (nodeId != Graphs::InvalidNodeId)
			{
				nodeIndexs.emplace_back(nodeId);
			}
		}
		//2. Create connections now that every node is created	
		//2 valid nodes -> 1 connection
		//3 valid nodes -> 3 connections
		if (nodeIndexs.size() == 2)
		{
			AddConnection(nodeIndexs[0], nodeIndexs[1]);
			AddConnection(nodeIndexs[1], nodeIndexs[0]);
		}
		else if (nodeIndexs.size() == 3)
		{
			AddConnection(nodeIndexs[0], nodeIndexs[1]);
			AddConnection(nodeIndexs[1], nodeIndexs[0]);
			
			AddConnection(nodeIndexs[2], nodeIndexs[1]);
			AddConnection(nodeIndexs[1], nodeIndexs[2]);
			
			AddConnection(nodeIndexs[0], nodeIndexs[2]);
			AddConnection(nodeIndexs[2], nodeIndexs[0]);
			
		}
	}
	//3. Set the connections cost to the actual distance
	SetConnectionCostsToDistances();
}
