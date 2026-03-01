#include "SpacePartitioning.h"
#include "DrawDebugHelpers.h"

static FVector2D GetAgentPos2D_World(const ASteeringAgent& Agent)
{
	const FVector L = Agent.GetActorLocation();
	return FVector2D(L.X, L.Y);
}

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
	const FVector2D agentPos = GetAgentPos2D_World(Agent);
	const int idx = PositionToIndex(agentPos);

	if (idx < 0 || idx >= (int)Cells.size())
	{
		UE_LOG(LogTemp, Error, TEXT("CellSpace::AddAgent OOB idx=%d pos=(%.1f,%.1f)"), idx, agentPos.X, agentPos.Y);
		return;
	}

	auto& list = Cells[idx].Agents;

	// guard na duplikaty
	if (std::find(list.begin(), list.end(), &Agent) == list.end())
	{
		list.push_back(&Agent);
	}
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	const FVector2D newPos = GetAgentPos2D_World(Agent);

	const int oldIndex = PositionToIndex(OldPos);
	const int newIndex = PositionToIndex(newPos);

	UE_LOG(LogTemp, Warning, TEXT("IDX old=%d new=%d | OldPos=(%.1f,%.1f) NewPos=(%.1f,%.1f)"),
		oldIndex, newIndex, OldPos.X, OldPos.Y, newPos.X, newPos.Y);

	if (oldIndex == newIndex)
		return;

	UE_LOG(LogTemp, Warning, TEXT("MOVE %d -> %d"), oldIndex, newIndex);

	if (oldIndex >= 0 && oldIndex < (int)Cells.size())
		Cells[oldIndex].Agents.remove(&Agent);

	if (newIndex >= 0 && newIndex < (int)Cells.size())
	{
		auto& list = Cells[newIndex].Agents;
		if (std::find(list.begin(), list.end(), &Agent) == list.end())
			list.push_back(&Agent);
	}
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius, bool bDebugThisAgent)
{
    NrOfNeighbors = 0;

    const FVector2D center = GetAgentPos2D_World(Agent);
    const float r = QueryRadius;
    const float rSq = r * r;

    // Query AABB
    FRect queryRect;
    queryRect.Min = FVector2D(center.X - r, center.Y - r);
    queryRect.Max = FVector2D(center.X + r, center.Y + r);

    // Convert world position to grid col/row
    auto PosToCol = [&](float x) -> int
    {
        float localX = x - CellOrigin.X;
        localX = FMath::Clamp(localX, 0.f, SpaceWidth - KINDA_SMALL_NUMBER);
        return FMath::Clamp(int(localX / CellWidth), 0, NrOfCols - 1);
    };

    auto PosToRow = [&](float y) -> int
    {
        float localY = y - CellOrigin.Y;
        localY = FMath::Clamp(localY, 0.f, SpaceHeight - KINDA_SMALL_NUMBER);
        return FMath::Clamp(int(localY / CellHeight), 0, NrOfRows - 1);
    };

    const int minCol = PosToCol(queryRect.Min.X);
    const int maxCol = PosToCol(queryRect.Max.X);
    const int minRow = PosToRow(queryRect.Min.Y);
    const int maxRow = PosToRow(queryRect.Max.Y);

    // --- DEBUG: zapisz info tylko dla debug agenta
    if (bDebugThisAgent)
    {
        DebugCheckedCellIndices.Reset();
        bHasDebugQuery = true;
        DebugQueryCenter = center;
        DebugQueryRadius = r;
    }

    // Iterate only the cells overlapped by the queryRect
    for (int row = minRow; row <= maxRow; ++row)
    {
        for (int col = minCol; col <= maxCol; ++col)
        {
            const int idx = row * NrOfCols + col;
            Cell& cell = Cells[idx];

            if (bDebugThisAgent)
            {
                DebugCheckedCellIndices.Add(idx);
            }

            if (!DoRectsOverlap(cell.BoundingBox, queryRect))
                continue;

            for (ASteeringAgent* other : cell.Agents)
            {
                if (!IsValid(other) || other == &Agent)
                    continue;

                const FVector2D otherPos = GetAgentPos2D_World(*other);
                const float distSq = (otherPos - center).SizeSquared();

                if (distSq <= rSq)
                {
                    if (NrOfNeighbors < Neighbors.Num())
                        Neighbors[NrOfNeighbors++] = other;
                    else
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

		// Z height so it's visible above the ground
		const float Z = 90.f;

		// Four corners of the cell rectangle
		const FVector p0(min.X, min.Y, Z);
		const FVector p1(max.X, min.Y, Z);
		const FVector p2(max.X, max.Y, Z);
		const FVector p3(min.X, max.Y, Z);

		// Highlight checked cells
		const bool bChecked = DebugCheckedCellIndices.Contains(i);
		const FColor lineColor = bChecked ? FColor::Yellow : FColor::Cyan;
		const float thickness = bChecked ? 3.f : 1.f;

		// Draw rectangle outline
		DrawDebugLine(pWorld, p0, p1, lineColor, false, 0.f, 0, thickness);
		DrawDebugLine(pWorld, p1, p2, lineColor, false, 0.f, 0, thickness);
		DrawDebugLine(pWorld, p2, p3, lineColor, false, 0.f, 0, thickness);
		DrawDebugLine(pWorld, p3, p0, lineColor, false, 0.f, 0, thickness);

		// Draw agent count text in the center of the cell
		const FVector2D center2D = (min + max) * 0.5f;
		const FVector center3D(center2D.X, center2D.Y, Z);

		const int count = static_cast<int>(cell.Agents.size());
		DrawDebugString(
			pWorld,
			center3D,
			FString::Printf(TEXT("%d"), count),
			nullptr,
			bChecked ? FColor::Yellow : FColor::White,
			0.f,   // lifetime (0 = one frame)
			false
		);
		
		// Draw debug circle only for the debug agent query (single circle)
		if (bHasDebugQuery && pWorld)
		{
			DrawDebugCircle(
				pWorld,
				FVector(DebugQueryCenter.X, DebugQueryCenter.Y, Z),
				DebugQueryRadius,
				64,
				FColor::Green,
				false,
				0.f,
				0,
				2.f,
				FVector(1, 0, 0),
				FVector(0, 1, 0),
				false
			);
		}
	}
}

int CellSpace::PositionToIndex(FVector2D const& Pos) const
{
	// pozycja lokalna względem originu siatki
	float localX = Pos.X - CellOrigin.X;
	float localY = Pos.Y - CellOrigin.Y;

	// clamp do wnętrza (ważne: SpaceWidth/Height są "max", więc odejmujemy mały eps)
	localX = FMath::Clamp(localX, 0.f, SpaceWidth  - KINDA_SMALL_NUMBER);
	localY = FMath::Clamp(localY, 0.f, SpaceHeight - KINDA_SMALL_NUMBER);

	const int col = FMath::Clamp(int(localX / CellWidth),  0, NrOfCols - 1);
	const int row = FMath::Clamp(int(localY / CellHeight), 0, NrOfRows - 1);

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