#include "SpacePartitioning.h"

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
	
	//calculate bounds of a cell
	CellWidth = Width / Cols;
	CellHeight = Height / Rows;

	for (int Row = 0; Row < Rows; ++Row)
	{
		for (int Col = 0; Col < Cols; ++Col)
		{
			Cells.emplace_back(Cell(CellWidth * Col, CellHeight* Row, CellWidth,CellHeight));
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
	if (oldIndex != newIndex)
	{
		//Remove from the old cell and add to the new cell
	}
	
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	// TODO Register the neighbors for the provided agent
	// TODO Only check the cells that are within the radius of the neighborhood
}

void CellSpace::EmptyCells()
{
	for (Cell& c : Cells)
		c.Agents.clear();
}

void CellSpace::RenderCells() const
{
	// TODO Render the cells with the number of agents inside of it
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