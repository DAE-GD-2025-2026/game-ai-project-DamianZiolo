#include "SpacePartitioning.h"
#include "DrawDebugHelpers.h"

// --- Cell ---
// ------------
Cell::Cell(float Left, float Bottom, float Width, float Height)
{
	BoundingBox.Min = { Left, Bottom };
	BoundingBox.Max = { BoundingBox.Min.X + Width, BoundingBox.Min.Y + Height };
}

std::vector<FVector2D> Cell::GetRectPoints() const
{
	const float left = BoundingBox.Min.X;
	const float bottom = BoundingBox.Min.Y;
	const float width = BoundingBox.Max.X - BoundingBox.Min.X;
	const float height = BoundingBox.Max.Y - BoundingBox.Min.Y;

	std::vector<FVector2D> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};

	return rectPoints;
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(UWorld* pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities)
	: pWorld{pWorld}
	, SpaceWidth{Width}
	, SpaceHeight{Height}
	, NrOfRows{Rows}
	, NrOfCols{Cols}
	, NrOfNeighbors{0}
{
	Neighbors.SetNum(MaxEntities);
	CellOrigin = FVector2D(-Width * 0.5f, -Height * 0.5f);
	//calculate bounds of a cell
	CellWidth = Width / Cols;
	CellHeight = Height / Rows;

	for (int Row = 0; Row < Rows; ++Row)
	{
		for (int Col = 0; Col < Cols; ++Col)
		{
			const float left = CellOrigin.X + CellWidth * Col;
			const float bottom = CellOrigin.Y + CellHeight * Row;
			Cells.emplace_back(Cell(left, bottom, CellWidth, CellHeight));
		}
	}
}

void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	const FVector2D agentPos = Agent.GetPosition();

	// Convert position -> cell index
	const int idx = PositionToIndex(agentPos);

	// If out of bounds, PositionToIndex should return -1
	if (idx < 0 || idx >= static_cast<int>(Cells.size()))
	{
		UE_LOG(LogTemp, Error, TEXT("CellSpace::AddAgent - Agent out of CellSpace bounds. Pos=(%.2f, %.2f), idx=%d, Cells=%d"),
			agentPos.X, agentPos.Y, idx, static_cast<int>(Cells.size()));
		//warn in editor but not crash in shipping
		ensureMsgf(false, TEXT("Agent position is outside CellSpace bounds."));
		return;
	}
	// Add agent pointer to that cell
	Cells[idx].Agents.push_back(&Agent);
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	//TODO Check if the agent needs to be moved to another cell.
	//TODO Use the calculated index for oldPos and currentPos for this
	int oldIndex = PositionToIndex(OldPos);
	int newIndex = PositionToIndex(Agent.GetPosition());

	//if agent is in the same cell, don't updateit
	if (oldIndex == newIndex)
		return;

	//if new position is outside of the grid, warning
	if (newIndex < 0 || newIndex >= static_cast<int>(Cells.size()))
	{
		const FVector2D pos = Agent.GetPosition();
		UE_LOG(LogTemp, Warning,
			TEXT("CellSpace::UpdateAgentCell - newIndex out of bounds. newIndex=%d pos=(%.2f,%.2f)"),
			newIndex, pos.X, pos.Y);
		return;
	}

	//remove from old cell
	if (oldIndex >= 0 && oldIndex < static_cast<int>(Cells.size()))
	{
		Cells[oldIndex].Agents.remove(&Agent);
	}

	//ad to new cell
	Cells[newIndex].Agents.emplace_back(&Agent);





	
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	// TODO Register the neighbors for the provided agent
	// TODO Only check the cells that are within the radius of the neighborhood

	// 1) Reset memory pool counter (no clear/push_back)
	NrOfNeighbors = 0;

	const FVector2D center = Agent.GetPosition();
	const float r = QueryRadius;
	const float rSq = r * r;

	// 2) Build query AABB (square around the query circle)
	FRect queryRect;
	queryRect.Min = FVector2D(center.X - r, center.Y - r);
	queryRect.Max = FVector2D(center.X + r, center.Y + r);

	// 3) Iterate cells, but only consider those whose bounding box overlaps the query rect
	for (Cell& cell : Cells)
	{
		// Skip cells that are definitely too far (no overlap)
		if (!DoRectsOverlap(cell.BoundingBox, queryRect))
			continue;

		// 4) Cell overlaps -> check actual agents inside this cell
		for (ASteeringAgent* other : cell.Agents)
		{
			if (!IsValid(other) || other == &Agent)
				continue;

			const FVector2D toOther = other->GetPosition() - center;
			const float distSq = toOther.SizeSquared();

			// 5) True neighborhood test (circle)
			if (distSq <= rSq)
			{
				// memory pool write (avoid out of bounds)
				if (NrOfNeighbors < Neighbors.Num())
				{
					Neighbors[NrOfNeighbors] = other;
					++NrOfNeighbors;
				}
				else
				{
					// pool full then stop early
					return;
				}
			}
		}
	}

}

void CellSpace::EmptyCells()
{
	for (Cell& c : Cells)
		c.Agents.clear();
}

void CellSpace::RenderCells() const
{
	if (!pWorld) return;

	// Draw each cell as a rectangle and print agent count inside
	for (int i = 0; i < static_cast<int>(Cells.size()); ++i)
	{
		const Cell& cell = Cells[i];

		const FVector2D min = cell.BoundingBox.Min;
		const FVector2D max = cell.BoundingBox.Max;

		// Z height so it's visible above the ground (tweak if needed)
		const float Z = 90.f;

		// Four corners of the cell rectangle
		const FVector p0(min.X, min.Y, Z);
		const FVector p1(max.X, min.Y, Z);
		const FVector p2(max.X, max.Y, Z);
		const FVector p3(min.X, max.Y, Z);

		// Draw rectangle outline
		DrawDebugLine(pWorld, p0, p1, FColor::Cyan, false, 0.f, 0, 1.f);
		DrawDebugLine(pWorld, p1, p2, FColor::Cyan, false, 0.f, 0, 1.f);
		DrawDebugLine(pWorld, p2, p3, FColor::Cyan, false, 0.f, 0, 1.f);
		DrawDebugLine(pWorld, p3, p0, FColor::Cyan, false, 0.f, 0, 1.f);

		// Draw agent count text in the center of the cell
		const FVector2D center2D = (min + max) * 0.5f;
		const FVector center3D(center2D.X, center2D.Y, Z);

		const int count = static_cast<int>(cell.Agents.size());
		DrawDebugString(
			pWorld,
			center3D,
			FString::Printf(TEXT("%d"), count),
			nullptr,
			FColor::White,
			0.f,   // lifetime (0 = one frame)
			false  // draw shadow
		);
	}
}

int CellSpace::PositionToIndex(FVector2D const& Pos) const
{
	//Move position relative to origin of the grid
	const float localX = Pos.X - CellOrigin.X;
	const float localY = Pos.Y - CellOrigin.Y;

	//If outside global bounds return invalid index
	if (localX < 0.f || localY < 0.f ||
		localX >= SpaceWidth || localY >= SpaceHeight)
	{
		return -1;
	}

	//Calculate column and row
	const int col = static_cast<int>(localX / CellWidth);
	const int row = static_cast<int>(localY / CellHeight);

	//Extra safety check
	if (col < 0 || col >= NrOfCols ||
		row < 0 || row >= NrOfRows)
	{
		return -1;
	}

	//Convert 2D index to 1D index
	return row * NrOfCols + col;
}

bool CellSpace::DoRectsOverlap(FRect const & RectA, FRect const & RectB)
{
	// Check if the rectangles are separated on either axis
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;
    
	// If they are not separated, they must overlap
	return true;
}