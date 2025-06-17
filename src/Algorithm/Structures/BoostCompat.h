//
// Created by User on 19.04.2025.
//

#ifndef BOOST_COMPAT_H
#define BOOST_COMPAT_H

#include <boost/geometry/geometries/concepts/point_concept.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/polygon/point_data.hpp>
#include <boost/polygon/segment_concept.hpp>
#include <boost/polygon/voronoi_diagram.hpp>
#include <utility>

// Можно заменить на функции-мапперы shapes::Polygon -> boost::polygon

using BoostSegment = std::pair<boost::polygon::point_data<float>, boost::polygon::point_data<float>>;

template <>
struct boost::polygon::geometry_concept<BoostSegment>
{
	using type = segment_concept;
};

template <>
struct boost::polygon::segment_traits<BoostSegment>
{
	using coordinate_type = float;
	using point_type = point_data<float>;

	static point_type get(const BoostSegment& segment, direction_1d dir)
	{
		return dir.to_int() ? segment.first : segment.second;
	}
};

namespace tp::boost_compat
{
using Graph = boost::adjacency_list<
	boost::listS,
	boost::vecS,
	boost::undirectedS,
	shapes::Point,
	boost::property<boost::edge_weight_t, double>>;

using Vertex = boost::graph_traits<Graph>::vertex_descriptor;

using Voronoi = boost::polygon::voronoi_diagram<double>;

using VoronoiVertex = Voronoi::vertex_type;
using VoronoiEdge = Voronoi::edge_type;

inline shapes::Point VertexToPoint(const VoronoiVertex& vertex)
{
	return { static_cast<float>(vertex.x()), static_cast<float>(vertex.y()) };
}

inline std::vector<shapes::Segment> GetAllEdges(const Graph& graph)
{
	std::vector<shapes::Segment> segments;

	Graph::edge_iterator ei, ei_end;
	for (std::tie(ei, ei_end) = boost::edges(graph); ei != ei_end; ++ei)
	{
		Vertex u = boost::source(*ei, graph);
		Vertex v = boost::target(*ei, graph);

		shapes::Point p1 = graph[u];
		shapes::Point p2 = graph[v];

		segments.emplace_back(p1, p2);
	}

	return segments;
}

class DijkstraShortestPath
{
public:
	explicit DijkstraShortestPath(const Graph& graph)
		: m_graph(graph)
		, m_predecessors(num_vertices(graph))
		, m_distances(num_vertices(graph))
	{
	}

	[[nodiscard]] std::vector<shapes::Point> FindPath(Vertex start, Vertex goal)
	{
		using namespace boost;
		dijkstra_shortest_paths(m_graph, start,
			predecessor_map(make_iterator_property_map(m_predecessors.begin(), get(vertex_index, m_graph)))
				.distance_map(make_iterator_property_map(m_distances.begin(), get(vertex_index, m_graph))));

		std::vector<shapes::Point> path;
		Vertex current = goal;
		while (current != start)
		{
			path.push_back(m_graph[current]);
			if (current == m_predecessors[current])
			{
				return {};
			}
			current = m_predecessors[current];
		}
		path.push_back(m_graph[start]);
		std::ranges::reverse(path);

		return path;
	}

private:
	Graph m_graph;
	std::vector<Vertex> m_predecessors;
	std::vector<double> m_distances;
};
} // namespace tp::boost_compat

#endif // BOOST_COMPAT_H
