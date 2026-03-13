#include "BFS.h"

#include <map>
#include <queue>

#include "Shared/Graph/Graph.h"

using namespace GameAI;

BFS::BFS(Graph* const pGraph)
	: pGraph(pGraph)
{
}

// TODO Breath First Search Algorithm searches for a path from the startNode to the destinationNode
std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
{
	std::vector<Node*> path;
	std::queue<Node*> queue;
	std::vector<Node*> visited;
	std::map<Node*, Node*> parent;

	queue.push(pStartNode);
	visited.push_back(pStartNode);

	Node* currentNode = nullptr;

	while (!queue.empty())
	{
		currentNode = queue.front();
		queue.pop();

		if (currentNode == pDestinationNode)
		{
			break;
		}

		auto connections = pGraph->FindConnectionsFrom(currentNode->GetId());

		for (auto connection : connections)
		{
			auto nextNode = pGraph->GetNode(connection->GetToId()).get();

			bool alreadyVisited = false;

			for (auto node : visited)
			{
				if (node == nextNode)
				{
					alreadyVisited = true;
					break;
				}
			}

			if (alreadyVisited)
				continue;

			visited.push_back(nextNode);
			parent[nextNode] = currentNode;
			queue.push(nextNode);
		}
	}

	// reconstruct path
	if (currentNode == pDestinationNode)
	{
		Node* node = pDestinationNode;

		while (node != pStartNode)
		{
			path.push_back(node);
			node = parent[node];
		}

		path.push_back(pStartNode);

		std::reverse(path.begin(), path.end());
	}

	return path;
}
