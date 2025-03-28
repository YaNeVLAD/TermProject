#pragma once

#include "VoronoiDiagram.h"
#include <vector>

namespace path_finder
{

class RefinedVoronoiDiargram
{
public:
	struct Cell
	{
		std::vector<VoronoiDiargram::Edge> Edges;
	};

	RefinedVoronoiDiargram(const VoronoiDiargram& diagram);

	std::vector<Cell> Get() const;

private:
	void ConstructRefinedVoronoi();
	Point FindAnchorPoint(const Point& point) const;
	void AddSupportingEdges(Cell& cell, const Point& anchor);

	std::vector<Cell> m_Cells;
	const VoronoiDiargram& m_Diagram;
};

} // namespace path_finder
