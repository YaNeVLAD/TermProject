#include <iostream>

#include "Window.h"

#include "PathFinder.h"

int main()
{
	try
	{
		path_finder::Point start(100, 100);
		path_finder::Point end(500, 500);

		tp::Window window(1920, 1080, "Term Project");
		std::vector<tp::Shapes::Segment> voronoiEdges;
		std::vector<tp::Shapes::Segment> intervalSegments;
		std::vector<tp::Shapes::Polygon> polygons;
		std::vector<path_finder::Obstacle> obstacles;
		path_finder::Obstacle currentObstacle;
		path_finder::IntervalGraph graph(obstacles);
		bool buildingObstacle = false;
		std::vector<tp::Shapes::Segment> graphEdges;

		window.SetMouseCallback([&](int button, int x, int y) {
			if (button == 0)
			{
				if (!buildingObstacle)
				{
					buildingObstacle = true;
					currentObstacle.Vertices().clear();
				}
				currentObstacle.Vertices().emplace_back(x, y);
				std::cout << "Added point: [" << x << ", " << y << "]" << std::endl;
			}
			else if (button == 1 && buildingObstacle)
			{
				if (currentObstacle.Vertices().size() >= 3)
				{
					obstacles.push_back(currentObstacle);
					std::cout << "Obstacle completed with " << currentObstacle.Vertices().size() << " points" << std::endl;

					polygons.clear();
					voronoiEdges.clear();
					intervalSegments.clear();
					path_finder::VoronoiDiargram vd(obstacles);

					graph.Build(vd, 0.1);
					auto path = graph.FindPath(start, end);
					// for (size_t i = 1; i < path.size(); ++i)
					//{
					//	graphEdges.push_back(tp::Shapes::Segment{
					//		{ static_cast<float>(path[i - 1].X), static_cast<float>(path[i - 1].Y) },
					//		{ static_cast<float>(path[i].X), static_cast<float>(path[i].Y) },
					//	});
					// }

					for (size_t i = 1; i < graph.nodes.size(); ++i)
					{
						graphEdges.push_back(tp::Shapes::Segment{
							{ static_cast<float>(graph.nodes[i - 1].position.X), static_cast<float>(graph.nodes[i - 1].position.Y) },
							{ static_cast<float>(graph.nodes[i].position.X), static_cast<float>(graph.nodes[i].position.Y) },
						});
					}

					// auto intervals = vd.DiscretizeEdges(1);

					// for (const auto& interval : intervals)
					//{
					//	for (size_t i = 1; i < interval.size(); ++i)
					//	{
					//		intervalSegments.emplace_back(
					//			tp::Shapes::Segment{
					//				{ static_cast<float>(interval[i - 1].position.X), static_cast<float>(interval[i - 1].position.Y) },
					//				{ static_cast<float>(interval[i].position.X), static_cast<float>(interval[i].position.Y) } });
					//	}
					// }

					for (auto& obs : obstacles)
					{
						tp::Shapes::Polygon poly;
						for (auto& point : obs.Vertices())
						{
							poly.emplace_back(static_cast<float>(point.X), static_cast<float>(point.Y));
						}
						polygons.push_back(poly);
					}

					for (auto& edge : vd.GetEdges())
					{
						voronoiEdges.emplace_back(
							tp::Shapes::Segment{
								{ static_cast<float>(edge.Start.X), static_cast<float>(edge.Start.Y) },
								{ static_cast<float>(edge.End.X), static_cast<float>(edge.End.Y) } });
					}
				}
				buildingObstacle = false;
				currentObstacle.Vertices().clear();
			}
			else if (button == 2)
			{
				start = path_finder::Point{ static_cast<double>(x), static_cast<double>(y) };
				std::cout << "Moved Start point: [" << x << ", " << y << "]" << std::endl;
			}
			else if (button == 4)
			{
				end = path_finder::Point{ static_cast<double>(x), static_cast<double>(y) };
				std::cout << "Moved End point: [" << x << ", " << y << "]" << std::endl;
			}
		});

		while (window.IsRunning())
		{
			if (buildingObstacle && !currentObstacle.Vertices().empty())
			{
				tp::Shapes::Polygon tempPoly;
				for (const auto& point : currentObstacle.Vertices())
				{
					tempPoly.emplace_back(static_cast<float>(point.X), static_cast<float>(point.Y));
				}
				window.Draw(tempPoly);
			}

			for (const auto& polygon : polygons)
			{
				window.Draw(polygon);
			}

			for (const auto& edge : graphEdges)
			{
				window.Draw(edge, sf::Color::Blue);
			}

			for (const auto& edge : voronoiEdges)
			{
				window.Draw(edge);
			}

			for (const auto& edge : intervalSegments)
			{
				window.Draw(edge, sf::Color::Red);
			}

			window.PollEvents();
			window.Render();
		}

		return EXIT_SUCCESS;
	}
	catch (const std::exception& ex)
	{
		std::cout << ex.what() << std::endl;
		return EXIT_FAILURE;
	}
}
