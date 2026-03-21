#pragma once
#include <vector>

#include "NavGraphPathfinding.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

namespace GameAI
{
	class SSFA final
	{
	public:

		static std::vector<NavLine> FindPortals(std::vector<Node*> const& Path, TriPolygon const& NavPoly)
		{
			std::vector<NavLine> Portals{};

			if (Path.empty())
				return Portals;

			// START
			NavLine startPortal;
			startPortal.P1 = Path.front()->GetPosition();
			startPortal.P2 = Path.front()->GetPosition();
			Portals.emplace_back(startPortal);

			for (int i = 0; i < Path.size(); ++i)
			{
				auto node = Path[i];
				int edgeIndex = static_cast<NavGraphNode*>(node)->GetEdgeIdx();

				if (edgeIndex == -1)
					continue;

				auto edge = NavPoly.GetEdges()[edgeIndex];

				FVector2D a = FVector2D(edge.GetP1(NavPoly));
				FVector2D b = FVector2D(edge.GetP2(NavPoly));

				// kierunek ścieżki
				FVector2D dir;

				if (i < Path.size() - 1)
					dir = (Path[i + 1]->GetPosition() - node->GetPosition()).GetSafeNormal();
				else
					dir = FVector2D(1, 0); // fallback

				FVector2D edgeVec = b - a;

				float cross = edgeVec.X * dir.Y - edgeVec.Y * dir.X;

				NavLine portal;

				if (cross >= 0)
				{
					portal.P1 = a; // RIGHT
					portal.P2 = b; // LEFT
				}
				else
				{
					portal.P1 = b;
					portal.P2 = a;
				}

				Portals.emplace_back(portal);
			}

			// END
			NavLine endPortal;
			endPortal.P1 = Path.back()->GetPosition();
			endPortal.P2 = Path.back()->GetPosition();
			Portals.emplace_back(endPortal);

			return Portals;
		}

static std::vector<FVector2D> OptimizePortals(std::vector<NavLine> const& Portals, TriPolygon const&)
{
    std::vector<FVector2D> Path{};

    if (Portals.empty())
        return Path;

    FVector2D apex = Portals[0].P1;

    FVector2D leftLeg  = Portals[0].P2 - apex;
    FVector2D rightLeg = Portals[0].P1 - apex;

    int apexIndex = 0;
    int leftLegIndex = 0;
    int rightLegIndex = 0;

    Path.push_back(apex);

    for (int portalIdx = 1; portalIdx < Portals.size(); ++portalIdx)
    {
        const NavLine& portal = Portals[portalIdx];

        // ===== RIGHT CHECK =====
        FVector2D newRight = portal.P1 - apex;
        float crossRight = rightLeg.X * newRight.Y - rightLeg.Y * newRight.X;

        if (crossRight <= 0)
        {
            float crossLeft = leftLeg.X * newRight.Y - leftLeg.Y * newRight.X;

            if (crossLeft < 0)
            {
                // SNAP TO LEFT
                apex = apex + leftLeg;
                Path.push_back(apex);

                apexIndex = leftLegIndex;
                portalIdx = apexIndex;

                leftLegIndex = apexIndex;
                rightLegIndex = apexIndex;

                rightLeg = Portals[rightLegIndex].P1 - apex;
                leftLeg  = Portals[leftLegIndex].P2 - apex;

                continue;
            }
            else
            {
                rightLeg = newRight;
                rightLegIndex = portalIdx;
            }
        }

        // ===== LEFT CHECK =====
        FVector2D newLeft = portal.P2 - apex;
        float crossLeftCheck = leftLeg.X * newLeft.Y - leftLeg.Y * newLeft.X;

        if (crossLeftCheck >= 0)
        {
            float crossRightCheck = rightLeg.X * newLeft.Y - rightLeg.Y * newLeft.X;

            if (crossRightCheck > 0)
            {
                // SNAP TO RIGHT
                apex = apex + rightLeg;
                Path.push_back(apex);

                apexIndex = rightLegIndex;
                portalIdx = apexIndex;

                leftLegIndex = apexIndex;
                rightLegIndex = apexIndex;

                rightLeg = Portals[rightLegIndex].P1 - apex;
                leftLeg  = Portals[leftLegIndex].P2 - apex;

                continue;
            }
            else
            {
                leftLeg = newLeft;
                leftLegIndex = portalIdx;
            }
        }
    }

    Path.push_back(Portals.back().P1);

    return Path;
}

	private:
		SSFA() {};
		~SSFA() {};
	};
}