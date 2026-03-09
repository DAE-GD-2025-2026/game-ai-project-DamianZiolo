#pragma once
#include <stack>
#include "Shared/Graph/Graph.h"

namespace GameAI
{
	enum class Eulerianity
	{
		notEulerian,
		semiEulerian,
		eulerian,
	};

	class EulerianPath final
	{
	public:
		EulerianPath(Graph* const pGraph);

		Eulerianity IsEulerian() const;
		std::vector<Node*> FindPath(Eulerianity& eulerianity) const;

	private:
		void VisitAllNodesDFS(const std::vector<Node*>& pNodes, std::vector<bool>& visited, int startIndex) const;
		bool IsConnected() const;

		Graph* m_pGraph;
	};

	inline EulerianPath::EulerianPath(Graph* const pGraph)
		: m_pGraph(pGraph)
	{
		
			
		
	}

	inline Eulerianity EulerianPath::IsEulerian() const
	{
		
		// TODO If the graph is not connected, there can be no Eulerian Trail
		if (!IsConnected())
			return Eulerianity::notEulerian;
		
		// TODO Count nodes with odd degree 
		auto Nodes = m_pGraph->GetActiveNodes();
		int oddDegree{ 0 };
		for (int i = 0; i < static_cast<int>(Nodes.size()); ++i)
		{
			int NodeId = Nodes[i]->GetId();
			auto fromConnections = m_pGraph->FindConnectionsFrom(NodeId);
			auto toConnections = m_pGraph->FindConnectionsTo(NodeId);
			
			int connections = static_cast<int> (fromConnections.size() + toConnections.size()) ;
			if (connections % 2 != 0)
				oddDegree++;
			
				
		}
		
		if (oddDegree > 2)
			return Eulerianity::notEulerian;
		if (oddDegree == 2)
			return Eulerianity::semiEulerian;
		if (oddDegree == 0)
			return Eulerianity::eulerian;
		
		
		// TODO A connected graph with more than 2 nodes with an odd degree (an odd amount of connections) is not Eulerian
		// TODO A connected graph with exactly 2 nodes with an odd degree is Semi-Eulerian (unless there are only 2 nodes)
		// TODO An Euler trail can be made, but only starting and ending in these 2 nodes
		// TODO A connected graph with no odd nodes is Eulerian
		
		return Eulerianity::notEulerian;
	}

	inline std::vector<Node*> EulerianPath::FindPath(Eulerianity& eulerianity) const
	{
		// Get a copy of the graph because this algorithm involves removing edges
		Graph graphCopy = m_pGraph->Clone();
		std::vector<Node*> Path = {};
		std::vector<Node*> Nodes = graphCopy.GetActiveNodes();
		int currentNodeId{ Graphs::InvalidNodeId };
		
		Eulerianity eulerian = IsEulerian();
		if (Nodes.empty())
			return Path;
		if (eulerian == Eulerianity::notEulerian)
			return Path;
		else if (eulerian == Eulerianity::semiEulerian)
		{
			for (int i = 0; i < static_cast<int>(Nodes.size()); ++i)
			{
				auto fromConnections = m_pGraph->FindConnectionsFrom(Nodes[i]->GetId());
				auto toConnections = m_pGraph->FindConnectionsTo(Nodes[i]->GetId());
				int connections = static_cast<int>(fromConnections.size() + toConnections.size()) ;
				if (connections % 2 != 0)
				{
					currentNodeId = Nodes[i]->GetId();
					break;
				}
			}
		}
		else if (eulerian == Eulerianity::eulerian)
		{
			currentNodeId = Nodes[0]->GetId();
		}
		
		
		// TODO Check if there can be an Euler path
		// TODO If this graph is not eulerian, return the empty path
		
		// TODO Start algorithm loop
		std::stack<int> nodeStack;

		std::reverse(Path.begin(), Path.end());
		return Path;
	}

	inline void EulerianPath::VisitAllNodesDFS(const std::vector<Node*>& Nodes, std::vector<bool>& visited, int startIndex ) const
	{
		// TODO Mark the visited node
		// TODO Ask the graph for the connections from that node
		// TODO recursively visit any valid connected nodes that were not visited before
		// TODO Tip: use an index-based for-loop to find the correct index
		visited[startIndex] = true;
		int currentNodeId{ Graphs::InvalidNodeId };
		currentNodeId = Nodes[startIndex]->GetId();
		auto connections = m_pGraph->FindConnectionsFrom(currentNodeId);
		for (int i = 0; i < static_cast<int>(connections.size()); ++i)
		{
			int nextNodeId = connections[i]->GetToId();
			for (int j = 0; j < static_cast<int>(Nodes.size()); ++j)
			{
				if (Nodes[j]->GetId() == nextNodeId)
				{
					if (!visited[j])
					{
						VisitAllNodesDFS(Nodes, visited, j);
					}
					break;
				}
					
			}
		}
		
		
		
		
	}

	inline bool EulerianPath::IsConnected() const
	{
		std::vector<Node*> Nodes = m_pGraph->GetActiveNodes();
		if (Nodes.size() == 0)
			return false;
		
		std::vector<bool> visited(Nodes.size(), false);
		int startIndex{ Graphs::InvalidNodeId }; //InvalidNodeId = -1, but just to be "more correct" or in case of change I'm taking it like this
		
		for (int index = 0; index < Nodes.size(); ++index)
		{
			auto fromConnections = m_pGraph->FindConnectionsFrom(Nodes[index]->GetId());
			auto toConnections = m_pGraph->FindConnectionsTo(Nodes[index]->GetId());
			
			if (fromConnections.empty() && toConnections.empty())
				return false;
			else
			{
				startIndex = index;
				break;
			}
		}
		
		if (startIndex == -1)
			return false;
		
		VisitAllNodesDFS(Nodes, visited, startIndex);
		
		for(int index = 0; index < Nodes.size(); ++index)
		{
			if (!visited[index])
				return false;
		}
		return true;
		
		
		// TODO choose a starting node
		// TODO start a depth-first-search traversal from the node that has at least one connection
		// TODO if a node was never visited, this graph is not connected
	}
}