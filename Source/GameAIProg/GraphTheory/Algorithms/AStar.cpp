#include "AStar.h"

using namespace GameAI;

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph)
	, HeuristicFunction(hFunction)
{
}

std::vector<Node*> AStar::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
    std::vector<Node*> path{};
    std::vector<NodeRecord> openList;
    std::vector<NodeRecord> closedList;

    // --- Kickstart ---
    NodeRecord startRecord;
    startRecord.pNode = pStartNode;
    startRecord.pConnection = nullptr;
    startRecord.costSoFar = 0;
    startRecord.estimatedTotalCost = GetHeuristicCost(pStartNode, pGoalNode);

    openList.push_back(startRecord);

    NodeRecord currentRecord;

    // --- Main loop ---
    while (!openList.empty())
    {
        // Find record with lowest f-cost
        currentRecord = openList[0];

        for (const NodeRecord& record : openList)
        {
            if (record.estimatedTotalCost < currentRecord.estimatedTotalCost)
            {
                currentRecord = record;
            }
        }

        // Goal reached
        if (currentRecord.pNode == pGoalNode)
        {
            break;
        }

        // Get connections
        auto connections = pGraph->FindConnectionsFrom(currentRecord.pNode->GetId());

        for (auto connection : connections)
        {
            Node* nextNode = pGraph->GetNode(connection->GetToId()).get();

            float gCost = currentRecord.costSoFar + connection->GetWeight();

            float hCost = GetHeuristicCost(nextNode, pGoalNode);
            float fCost = gCost + hCost;

            // --- Check closedList ---
            bool skipNode = false;

            for (auto& record : closedList)
            {
                if (record.pNode == nextNode)
                {
                    if (record.costSoFar <= gCost)
                    {
                        skipNode = true;
                        break;
                    }
                    else
                    {
                        closedList.erase(
                            std::remove(closedList.begin(), closedList.end(), record),
                            closedList.end());
                        break;
                    }
                }
            }

            if (skipNode)
                continue;

            // --- Check openList ---
            for (auto& record : openList)
            {
                if (record.pNode == nextNode)
                {
                    if (record.costSoFar <= gCost)
                    {
                        skipNode = true;
                        break;
                    }
                    else
                    {
                        openList.erase(
                            std::remove(openList.begin(), openList.end(), record),
                            openList.end());
                        break;
                    }
                }
            }

            if (skipNode)
                continue;

            // Create new record
            NodeRecord newRecord;
            newRecord.pNode = nextNode;
            newRecord.pConnection = connection;
            newRecord.costSoFar = gCost;
            newRecord.estimatedTotalCost = fCost;

            openList.push_back(newRecord);
        }

        // Move currentRecord to closedList
        openList.erase(
            std::remove(openList.begin(), openList.end(), currentRecord),
            openList.end());

        closedList.push_back(currentRecord);
    }

    // --- No path ---
    if (currentRecord.pNode != pGoalNode)
    {
        return path;
    }

    // --- Backtracking ---
    while (currentRecord.pNode != pStartNode)
    {
        path.push_back(currentRecord.pNode);

        int fromNodeId = currentRecord.pConnection->GetFromId();

        for (auto& record : closedList)
        {
            if (record.pNode->GetId() == fromNodeId)
            {
                currentRecord = record;
                break;
            }
        }
    }

    path.push_back(pStartNode);

    std::reverse(path.begin(), path.end());

    return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
	FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition() - pGraph->GetNode(pStartNode->GetId())->GetPosition();
	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}