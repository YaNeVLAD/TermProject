#pragma once

#include <vector>

#include "VoronoiDiagram.h"

namespace path_finder
{

class IntervalGraph
{
public:
	IntervalGraph(std::vector<Obstacle>& obstacles)
		: m_Obstacles(obstacles)
	{
	}

	struct Edge
	{
		size_t from;
		size_t to;
		double weight;
	};

	void Build(const VoronoiDiargram& vd, double epsilon)
	{
		m_Obstacles = vd.GetObstacles();
		m_Intervals = vd.DiscretizeEdges(epsilon);

		for (size_t edge_idx = 0; edge_idx < m_Intervals.size(); ++edge_idx)
		{
			for (const auto& point : m_Intervals[edge_idx])
			{
				nodes.push_back({ point.position,
					point.clearance,
					edge_idx,
					&vd.GetEdges()[edge_idx] });
			}
		}

		ConnectIntervals(vd);
		BuildAdjacencyList();
	}
	std::vector<GraphNode> nodes;
	std::vector<Point> FindPath(const Point& start, const Point& goal);

private:
	double m_delta = 0.1;

	std::vector<Obstacle>& m_Obstacles;

	std::vector<Edge> edges;
	std::vector<std::vector<size_t>> adjacency_list;
	std::vector<std::vector<GraphNode>> m_Intervals;

	void AddNode(const GraphNode& node);

	const Obstacle* FindNearestObstacle(const Point& p) const;
	void ConnectIntervals(const VoronoiDiargram& vd);
	std::pair<std::vector<Point>, double> CalculateConnection(const GraphNode& a, const GraphNode& b);
	std::pair<std::vector<Point>, double> CalculateLinearSegment(const GraphNode& a, const GraphNode& b);
	std::pair<std::vector<Point>, double> CalculateSpiralArc(const GraphNode& a, const GraphNode& b);
	std::pair<std::vector<Point>, double> CalculateCircularArc(const GraphNode& a, const GraphNode& b);
	size_t FindNearestNode(const Point& p) const;
	bool IsPathClear(const std::vector<Point>& path) const;
	void BuildAdjacencyList();
	double GetClearance(const Point& p) const;
	double CalculateSpiralWeight(const std::vector<Point>& path, double delta);
};

} // namespace path_finder
