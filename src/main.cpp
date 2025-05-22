#include <algorithm>
#include <format>
#include <iomanip>
#include <iostream>
#include <vector>

#include "Algorithm/HaloGenerator/HaloGenerator.h"
#include "Algorithm/PathFinder/PathFinder.h"
#include "Algorithm/Voronoi/Voronoi.h"
#include "Utility/Logger/Logger.h"
#include "Window/Window.h"

// В экранных координатах (Y вниз) результат > 0 означает CW порядок, < 0 - CCW.
float computeSignedArea(const tp::shapes::Polygon& polygon)
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
	std::vector<Polygon> halos;
	std::vector<Polygon> obstacles;
	VoronoiData voronoiData;
	std::vector<Segment> path;
	bool isBuilding = false;

	Point start = { 100, 500 };
	Point end = { 1000, 500 };

	Window window(1024, 720, "Term");

	HaloGenerator hg(10.f, 4);

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
			voronoiData = vd.Generate(obstacles, halos, start, end);

			PathFinder pathFinder(voronoiData.refined_edges, obstacles, start, end, voronoiData.N_s, voronoiData.N_t);
			path = pathFinder.GetPath();
		}
		if (button == 3)
		{
			end = clickPos;
			Voronoi vd(window.GetBounds());
			voronoiData = vd.Generate(obstacles, halos, start, end);

			PathFinder pathFinder(voronoiData.refined_edges, obstacles, start, end, voronoiData.N_s, voronoiData.N_t);
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
				// !!! HALO GENERATION START !!!
				if (currentObstacle.size() < 3)
				{
					return;
				}
				obstacles.push_back(currentObstacle);
				LOG_DEBUG(std::format("Obstacle created with {} points", currentObstacle.size()));

				Polygon obstacleForHalo = currentObstacle;

				// На вход всегда подаём CCW полигон
				if (float area = computeSignedArea(obstacleForHalo); area > DBL_EPSILON)
				{
					LOG_DEBUG(std::format("Input obstacle area: {} (CW), reversing to CCW", area));
					std::ranges::reverse(obstacleForHalo);
				}

				// Генерируем ореол (ожидаем CCW результат)
				Polygon newHalo = hg.Generate(obstacleForHalo);
				LOG_DEBUG(std::format("Generated halo with {} points", newHalo.size()));

				halos.push_back(newHalo);
				// !!! HALO GENERATION END !!!

				// !!! VORONOI GENERATION START !!!
				Voronoi vd(window.GetBounds());
				voronoiData = vd.Generate(obstacles, halos, start, end);

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
			PathFinder pathFinder(voronoiData.refined_edges, obstacles, start, end, voronoiData.N_s, voronoiData.N_t);
			path = pathFinder.GetPath();
			// !!! PATH FINDING END !!!
		}
		if (key == keyboard::Key::F5)
		{
			// !!! VORONOI GENERATION START !!!
			Voronoi vd(window.GetBounds());
			voronoiData = vd.Generate(obstacles, halos, start, end);
			// !!! VORONOI GENERATION END !!!
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

		for (const auto& halo : halos)
		{
			window.Draw(halo, haloColor, false);
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
