#pragma once

#include <boost/polygon/voronoi.hpp>

#include "Point.h"

namespace path_finder
{
struct Segment;
struct GraphNode;

class VoronoiDiargram
{
public:
	struct Edge
	{
		Point Start, End;

		enum class FeatureType
		{
			Unknown = 0,
			Vertex,
			Edge,
		};

		const boost::polygon::voronoi_edge<double>* boost_edge = nullptr;

		FeatureType LeftFeature, RightFeature;

		const Obstacle* Left;
		const Obstacle* Right;

		double GetClearanceAt(const Point& point) const;
	};

	VoronoiDiargram(std::vector<Obstacle>& obstacles);
	std::vector<Edge> GetEdges() const;
	std::vector<Obstacle>& GetObstacles() const;
	std::vector<std::vector<GraphNode>> DiscretizeEdges(double epsilon) const;
	std::vector<std::pair<size_t, size_t>> GetAdjacentIntervals() const;

private:
	void ConstructVoronoi();
	void AnnotateEdges();

	bool SharesCell(const Edge& edge, const Obstacle* obstacle) const;

	std::vector<Obstacle>& m_Obstacles;
	std::vector<Edge> m_Edges;
};

struct GraphNode
{
	Point position;
	double clearance;
	size_t edge_index;
	const VoronoiDiargram::Edge* edge;
	const Obstacle* nearest_obstacle;
	bool is_temporary = false;

	GraphNode operator*(double scalar) const
	{
		return { position * scalar, clearance, edge_index, edge };
	}
};
} // namespace path_finder
