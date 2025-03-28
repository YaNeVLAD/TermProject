#include "VoronoiDiagram.h"

struct GraphNode
{
	path_finder::Point position;
	const path_finder::VoronoiDiagram::Edge* edge;
};

struct Edge
{
	size_t from;
	size_t to;
	double weight;
};

class IntervalGraph
{
public:
	void Build(const path_finder::VoronoiDiagram& vd, double spacing);
	std::vector<path_finder::Point> FindPath(const path_finder::Point& start, const path_finder::Point& goal);

private:
	std::vector<GraphNode> nodes;
	std::vector<Edge> edges;
	std::vector<path_finder::Obstacle> obstacles;

	void AddNodesAlongEdge(const path_finder::VoronoiDiagram::Edge& edge, double spacing);
	void ConnectAdjacentNodesOnEdge(size_t start_idx, size_t end_idx);
	bool IsStraightPathValid(const path_finder::Point& a, const path_finder::Point& b) const;
};
