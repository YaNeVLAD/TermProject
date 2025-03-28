#pragma once

#include "Point.h"

namespace path_finder
{
class VVComplex
{
#pragma once

public:
	struct Edge
	{
		Point Start, End;
	};

	VVComplex(std::vector<Obstacle>& obstacles);
	std::vector<Edge> Get() const;
	std::vector<Obstacle>& GetObstacles() const;

private:
	void ConstructComplex();
	bool CheckVisibility(const Point& p1, const Point& p2, const std::vector<Obstacle>& obstacles);

	std::vector<Obstacle>& m_Obstacles;
	std::vector<Edge> m_Edges;
};

} // namespace path_finder
