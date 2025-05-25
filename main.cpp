#include <algorithm>
#include <format>
#include <iomanip>
#include <iostream>
#include <vector>

#include "src/Algorithm/HaloGenerator/HaloGenerator.h"
#include "src/Algorithm/PathFinder/PathFinder.h"
#include "src/Algorithm/Voronoi/Voronoi.h"
#include "src/Utility/Logger/Logger.h"
#include "src/Window/Window.h"
#include <memory>
// В экранных координатах (Y вниз) результат > 0 означает CW порядок, < 0 - CCW.
float ComputeSignedArea(const tp::shapes::Polygon& polygon)
{
	float area = 0.0f;
	size_t n = polygon.size();
	if (n < 3)
	{
		return 0.0f;
	}

	for (size_t i = 0; i < n; ++i)
	{
		const auto& [ax, ay] = polygon[i];
		const auto& [bx, by] = polygon[(i + 1) % n];
		area += ax * by - bx * ay; // Формула шнуровки
	}
	return area / 2.0f;
}

// Подумать как сделать последовательные аппроксимации через стратегию

int main()
{
	using namespace tp;
	using namespace tp::shapes;
	using namespace tp::halos;

	std::vector<Point> currentObstacle;
	std::vector<Polygon> obstacles;
	VoronoiData voronoiData;
	std::vector<Segment> path;
	bool isBuilding = false;

	auto currGraphType = GraphType::G1;

	Point start = { 100, 500 };
	Point end = { 1000, 500 };

	Window window(1024, 720, "Term");

	auto edgeColor = Color{ 0x246AF3FF };
	auto pathColor = Color::Green;
	auto haloColor = Color::Red;
	auto obstacleColor = Color{ 0xFF603DFF };
	auto buildingColor = Color::Green;
	auto backgroundColor = Color::White;

	std::cout << std::fixed << std::setprecision(2);

	window.SetMouseCallback([&](int button, int x, int y) {
		Point clickPos = { static_cast<float>(x), static_cast<float>(y) };
		if (button == 2)
		{
			start = clickPos;
			Voronoi vd(window.GetBounds());
			voronoiData = vd.Generate(obstacles, start, end);

			PathFinder pathFinder(currGraphType, voronoiData, obstacles, start, end);
			path = pathFinder.GetPath();
		}
		if (button == 3)
		{
			end = clickPos;
			Voronoi vd(window.GetBounds());
			voronoiData = vd.Generate(obstacles, start, end);

			PathFinder pathFinder(currGraphType, voronoiData, obstacles, start, end);
			path = pathFinder.GetPath();
		}
		if (button == 0)
		{
			if (!isBuilding)
			{
				isBuilding = true;
				currentObstacle.clear();
				LOG_DEBUG(std::format("Started building obstacle {}", obstacles.size()));
			}
			currentObstacle.push_back(clickPos);
			LOG_DEBUG(std::format("Added Point: [{}, {}]", clickPos.x, clickPos.y));
		}
		if (button == 1 && isBuilding) // ПКМ - Завершить фигуру
		{
			if (!currentObstacle.empty())
			{
				if (currentObstacle.size() < 3)
				{
					return currentObstacle.clear();
				}
				obstacles.push_back(currentObstacle);
				LOG_DEBUG(std::format("Obstacle created with {} points", currentObstacle.size()));

				// !!! VORONOI GENERATION START !!!
				Voronoi vd(window.GetBounds());
				voronoiData = vd.Generate(obstacles, start, end);

				// !!! VORONOI GENERATION END !!!
			}

			isBuilding = false;
			currentObstacle.clear();
		}
	});

	window.SetKeyboardCallback([&](keyboard::Key key) {
		if (key == keyboard::Key::Escape)
		{
			window.Close();
		}

		if (key == keyboard::Key::Space)
		{
			// !!! PATH FINDING START !!!
			PathFinder pathFinder(currGraphType, voronoiData, obstacles, start, end);
			path = pathFinder.GetPath();
			// !!! PATH FINDING END !!!
		}
		if (key == keyboard::Key::F5)
		{
			// !!! VORONOI GENERATION START !!!
			Voronoi vd(window.GetBounds());
			voronoiData = vd.Generate(obstacles, start, end);
			// !!! VORONOI GENERATION END !!!
		}
		if (key == keyboard::Key::F1)
		{
			currGraphType = currGraphType == GraphType::G1 ? GraphType::G2 : GraphType::G1;

			LOG_DEBUG(std::format("Current graph type: {}", (int)currGraphType));

			PathFinder pathFinder(currGraphType, voronoiData, obstacles, start, end);
			path = pathFinder.GetPath();
		}
	});

	while (window.IsRunning())
	{
		window.PollEvents();

		window.Draw(start, Color::Magenta);
		window.Draw(end, Color::Magenta);

		for (const auto& edge : voronoiData.refined_edges)
		{
			window.Draw(edge, edgeColor);
		}

		for (const auto& seg : path)
		{
			window.Draw(seg, pathColor);
		}

		for (const auto& obstacle : obstacles)
		{
			if (obstacle.size() == 1)
			{
				window.Draw(obstacle[0], obstacleColor);
			}
			else if (obstacle.size() == 2)
			{
				window.Draw(Segment{ obstacle[0], obstacle[1] }, obstacleColor);
			}
			else if (obstacle.size() > 2)
			{
				window.Draw(obstacle, obstacleColor);
			}
		}

		if (isBuilding && !currentObstacle.empty())
		{
			if (currentObstacle.size() == 1)
			{
				window.Draw(currentObstacle[0], buildingColor);
			}
			else
			{
				if (currentObstacle.size() == 1)
				{
					window.Draw(currentObstacle[0], buildingColor);
				}
				else if (currentObstacle.size() == 2)
				{
					window.Draw(Segment{ currentObstacle[0], currentObstacle[1] }, buildingColor);
				}
				else
				{
					window.Draw(currentObstacle, buildingColor);
				}
			}
		}

		window.Render(backgroundColor);
	}

	return 0;
}
