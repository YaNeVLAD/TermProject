#include "VVComplex.h"

#include <utility>

#include <boost/polygon/point_concept.hpp>
#include <boost/polygon/segment_concept.hpp>
#include <boost/polygon/voronoi.hpp>

namespace path_finder
{
using BoostSegment = std::pair<boost::polygon::point_data<double>, boost::polygon::point_data<double>>;
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

VVComplex::VVComplex(std::vector<Obstacle>& obstacles)
	: m_Obstacles(obstacles)
{
	ConstructComplex();
}

std::vector<VVComplex::Edge> VVComplex::Get() const
{
	return m_Edges;
}

std::vector<Obstacle>& VVComplex::GetObstacles() const
{
	return m_Obstacles;
}

void VVComplex::ConstructComplex()
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

		bool isInsideAnyObstacle = false;
		Point midPoint{ (start.X + end.X) / 2, (start.Y + end.Y) / 2 };

		for (const auto& obstacle : m_Obstacles)
		{
			if (obstacle.Contains(midPoint))
			{
				isInsideAnyObstacle = true;
				break;
			}
		}

		if (!isInsideAnyObstacle && start != end && CheckVisibility(start, end, m_Obstacles))
		{
			m_Edges.emplace_back(start, end);
		}
	}
}

bool VVComplex::CheckVisibility(const Point& p1, const Point& p2, const std::vector<Obstacle>& obstacles)
{
	for (const auto& obstacle : obstacles)
	{
		if (obstacle.Intersects(p1, p2))
		{
			return false;
		}
	}
	return true;
}

} // namespace path_finder
