#include "RefinedVoronoiDiargram.h"
#include <map>

namespace path_finder
{

static double Distance(Point first, Point second)
{
	double dx = second.X - first.X;
	double dy = second.Y - first.Y;

	return dx * dx + dy * dy;
}

RefinedVoronoiDiargram::RefinedVoronoiDiargram(const VoronoiDiargram& diagram)
	: m_Diagram(diagram)
{
	ConstructRefinedVoronoi();
}

std::vector<RefinedVoronoiDiargram::Cell> RefinedVoronoiDiargram::Get() const
{
	return m_Cells;
}

void RefinedVoronoiDiargram::ConstructRefinedVoronoi()
{
	std::map<Point, Cell> cellMap;
	for (const auto& edge : m_Diagram.GetEdges())
	{
		cellMap[edge.Start].Edges.push_back(edge);
		cellMap[edge.End].Edges.push_back(edge);
	}

	for (auto& [point, cell] : cellMap)
	{
		Point anchor = FindAnchorPoint(point);
		AddSupportingEdges(cell, anchor);
		m_Cells.push_back(cell);
	}
}

Point RefinedVoronoiDiargram::FindAnchorPoint(const Point& point) const
{
	Point bestAnchor = point;
	double minDist = std::numeric_limits<double>::max();

	for (auto& obstacle : m_Diagram.GetObstacles())
	{
		for (auto& vertex : obstacle.Vertices())
		{
			double dist = Distance(point, vertex);
			if (dist < minDist)
			{
				minDist = dist;
				bestAnchor = vertex;
			}
		}
	}
	return bestAnchor;
}

void RefinedVoronoiDiargram::AddSupportingEdges(Cell& cell, const Point& anchor)
{
	for (auto& edge : cell.Edges)
	{
		Point mid{ (edge.Start.X + edge.End.X) / 2, (edge.Start.Y + edge.End.Y) / 2 };
		cell.Edges.emplace_back(mid, anchor);
	}
}

} // namespace path_finder
