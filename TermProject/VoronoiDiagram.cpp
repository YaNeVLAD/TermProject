#include "VoronoiDiagram.h"

#include "Details.hpp"

#include <boost/polygon/point_concept.hpp>
#include <boost/polygon/segment_concept.hpp>
#include <boost/polygon/voronoi.hpp>

namespace path_finder
{
using BoostSegment = std::pair<boost::polygon::point_data<double>, boost::polygon::point_data<double>>;

double VoronoiDiargram::Edge::GetClearanceAt(const Point& point) const
{
	double left_dist = std::numeric_limits<double>::max();
	double right_dist = std::numeric_limits<double>::max();

	if (Left)
	{
		left_dist = (point - Left->ClosestPoint(point)).Length();
	}
	if (Right)
	{
		right_dist = (point - Right->ClosestPoint(point)).Length();
	}

	return std::min(left_dist, right_dist);
}
} // namespace path_finder

namespace boost::polygon
{

template <>
struct geometry_concept<path_finder::BoostSegment>
{
	using type = segment_concept;
};

template <>
struct segment_traits<path_finder::BoostSegment>
{
	using coordinate_type = double;
	using point_type = point_data<double>;

	static point_type get(const path_finder::BoostSegment& segment, direction_1d dir)
	{
		return dir.to_int() ? segment.first : segment.second;
	}
};

} // namespace boost::polygon

namespace path_finder
{

VoronoiDiargram::VoronoiDiargram(std::vector<Obstacle>& obstacles)
	: m_Obstacles(obstacles)
{
	ConstructVoronoi();
}

std::vector<VoronoiDiargram::Edge> VoronoiDiargram::GetEdges() const
{
	return m_Edges;
}

std::vector<Obstacle>& VoronoiDiargram::GetObstacles() const
{
	return m_Obstacles;
}

std::vector<std::vector<GraphNode>> VoronoiDiargram::DiscretizeEdges(double epsilon) const
{
	std::vector<std::vector<GraphNode>> intervals;
	double clearance = 0;
	for (const auto& edge : m_Edges)
	{
		std::vector<GraphNode> nodes;
		Point dir = edge.End - edge.Start;
		double length = dir.Length();
		dir.Normalize();

		double t = 0.0;
		while (t < length)
		{
			Point current = edge.Start + dir * t;
			clearance = edge.GetClearanceAt(current);
			double step = clearance * epsilon;

			nodes.push_back({ current, clearance, 0, &edge });
			t += step;
		}
		nodes.push_back({ edge.End, clearance, 0, nullptr });
		intervals.push_back(nodes);
	}
	return intervals;
}

std::vector<std::pair<size_t, size_t>> VoronoiDiargram::GetAdjacentIntervals() const
{
	std::vector<std::pair<size_t, size_t>> adjacents;

	for (size_t i = 0; i < m_Edges.size(); ++i)
	{
		const auto& edge = m_Edges[i];

		for (size_t j = i + 1; j < m_Edges.size(); ++j)
		{
			if (SharesCell(m_Edges[j], edge.Left) || SharesCell(m_Edges[j], edge.Right))
			{
				adjacents.emplace_back(i, j);
			}
		}
	}
	return adjacents;
}

void VoronoiDiargram::ConstructVoronoi()
{
	using namespace boost::polygon;

	m_Edges.clear();
	std::vector<BoostSegment> segments;

	const unsigned left = 0, right = 1920, top = 0, bottom = 1080;
	segments.emplace_back(point_data<int>(left, top), point_data<int>(right, top));
	segments.emplace_back(point_data<int>(right, top), point_data<int>(right, bottom));
	segments.emplace_back(point_data<int>(right, bottom), point_data<int>(left, bottom));
	segments.emplace_back(point_data<int>(left, bottom), point_data<int>(left, top));

	for (auto& obstacle : m_Obstacles)
	{
		auto& vertices = obstacle.Vertices();
		for (size_t i = 0; i < vertices.size(); ++i)
		{
			point_data<int> p1(vertices[i].X, vertices[i].Y);
			point_data<int> p2(vertices[(i + 1) % vertices.size()].X,
				vertices[(i + 1) % vertices.size()].Y);
			segments.emplace_back(p1, p2);
		}
	}

	voronoi_diagram<double> vd;
	construct_voronoi(segments.begin(), segments.end(), &vd);

	for (const auto& edge : vd.edges())
	{
		if (!edge.is_finite())
		{
			continue;
		}

		auto* v0 = edge.vertex0();
		auto* v1 = edge.vertex1();
		if (!v0 || !v1)
		{
			continue;
		}

		Point start{ v0->x(), v0->y() };
		Point end{ v1->x(), v1->y() };

		bool insideAnyObstacle = false;
		Point midPoint{ (start.X + end.X) / 2, (start.Y + end.Y) / 2 };

		for (const auto& obstacle : m_Obstacles)
		{
			if (obstacle.Contains(midPoint))
			{
				insideAnyObstacle = true;
				break;
			}
		}

		if (!insideAnyObstacle && start != end)
		{
			auto& it = m_Edges.emplace_back(start, end);
			it.boost_edge = &edge;
		}
	}

	AnnotateEdges();
}

void VoronoiDiargram::AnnotateEdges()
{
	using namespace boost::polygon;

	for (auto& edge : m_Edges)
	{
		if (!edge.boost_edge)
			continue;

		const auto& bcell0 = *edge.boost_edge->cell();
		const auto& bcell1 = *edge.boost_edge->twin()->cell();

		edge.LeftFeature = (bcell0.contains_point()) ? Edge::FeatureType::Vertex : Edge::FeatureType::Edge;

		edge.RightFeature = (bcell1.contains_point()) ? Edge::FeatureType::Vertex : Edge::FeatureType::Edge;

		if (bcell0.source_index() < m_Obstacles.size())
		{
			edge.Left = &m_Obstacles[bcell0.source_index()];
		}
		if (bcell1.source_index() < m_Obstacles.size())
		{
			edge.Right = &m_Obstacles[bcell1.source_index()];
		}
	}
}

bool VoronoiDiargram::SharesCell(const Edge& edge, const Obstacle* obstacle) const
{
	return (edge.Left == obstacle) || (edge.Right == obstacle);
}

} // namespace path_finder
