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

// Добавить рамку препятствия, внутри будем всё искать

int main()
{
	using namespace tp;
	using namespace tp::shapes;
	using namespace tp::halos;

	Polygon currentObstacle;
	std::vector<Polygon> obstacles;
	VoronoiData voronoiData;
	std::vector<Segment> path;
	std::vector<Segment> edges;
	bool isBuilding = false;

	auto currGraphType = GraphType::G1;

	Point start = { 100, 500 };
	Point end = { 1000, 500 };

	Window window(1920, 1080, "Term");

	auto edgeColor = Color{ 0x246AF3FF };
	auto pathColor = Color::Green;
	auto obstacleColor = Color{ 0xFF603DFF };
	auto buildingColor = Color::Green;
	auto backgroundColor = Color::White;

	window.SetMouseCallback([&](int button, int x, int y) {
		Point clickPos = { static_cast<float>(x), static_cast<float>(y) };
		if (button == 2)
		{
			start = clickPos;
		}
		if (button == 3)
		{
			end = clickPos;
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
				voronoiData = vd.Generate(obstacles, start, end, 8);

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
			Voronoi vd(window.GetBounds());
			voronoiData = vd.Generate(obstacles, start, end, 8);

			PathFinder pathFinder(currGraphType, voronoiData, obstacles, start, end);
			path = pathFinder.GetPath();
			edges = pathFinder.GetEdges();
			// !!! PATH FINDING END !!!
		}
		if (key == keyboard::Key::F5)
		{
			// !!! VORONOI GENERATION START !!!
			Voronoi vd(window.GetBounds());
			voronoiData = vd.Generate(obstacles, start, end, 8);

			PathFinder pathFinder(currGraphType, voronoiData, obstacles, start, end);
			edges = pathFinder.GetEdges();
			path = pathFinder.GetPath();
			// !!! VORONOI GENERATION END !!!
		}
		if (key == keyboard::Key::F1)
		{
			currGraphType = currGraphType == GraphType::G1 ? GraphType::G2 : GraphType::G1;

			LOG_DEBUG(std::format("Current graph type: {}", static_cast<int>(currGraphType)));

			PathFinder pathFinder(currGraphType, voronoiData, obstacles, start, end);
			path = pathFinder.GetPath();
			edges = pathFinder.GetEdges();
		}
	});

	while (window.IsRunning())
	{
		window.PollEvents();

		window.Draw(start, Color::Magenta);
		window.Draw(end, Color::Magenta);

		// for (const auto& cell : voronoiData.all_tilde_V_cells)
		//{
		//	window.Draw(cell, Color(0, 255, 0, 64));
		// }

		for (const auto& edge : voronoiData.refined_edges)
		{
			window.Draw(edge, edge.color);
		}

		for (const auto& seg : edges)
		{
			window.Draw(seg, Color::Magenta);
		}

		for (const auto& seg : path)
		{
			window.Draw(seg, pathColor);
		}

		for (const auto& obstacle : obstacles)
		{
			window.Draw(obstacle, obstacleColor);
		}

		if (voronoiData.s_cell_idx != -1)
		{
			window.Draw(voronoiData.all_tilde_V_cells[voronoiData.s_cell_idx], Color(0, 255, 0, 64));
		}
		if (voronoiData.t_cell_idx != -1)
		{
			window.Draw(voronoiData.all_tilde_V_cells[voronoiData.t_cell_idx], Color(0, 255, 0, 64));
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
					window.Draw(currentObstacle.front(), buildingColor);
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
