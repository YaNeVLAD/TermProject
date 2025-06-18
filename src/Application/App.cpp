#include "App.h"

#include <SFML/Graphics.hpp>
#include <imgui.h>

#include "../Algorithm/Math/Math.hpp"
#include "../Algorithm/PathFinder/PathFinder.h"
#include "../Algorithm/Voronoi/Voronoi.h"
#include "../Utility/Logger/Logger.h"
#include "../Utility/Shapes.hpp"

using namespace tp;
using namespace tp::shapes;

namespace
{
BorderRect GetAdjustedWindowSize(const Window& window, const AppState& state)
{
	BorderRect rect = window.GetBounds();
	rect.right -= state.settingsWindowWidth;

	return rect;
}

void GeneratePath(AppState& state, PathFinder& pathFinder)
{
	state.path = pathFinder.FindPath(state.currGraphType, state.voronoiData, state.obstacles, state.start, state.end);
	state.edges = pathFinder.GetEdges();
}

void RefreshAll(AppState& state, PathFinder& pathFinder, Voronoi& voronoi)
{
	state.voronoiData = voronoi.Generate(state.obstacles, state.start, state.end, state.segmentsPerArc);
	GeneratePath(state, pathFinder);
}
} // namespace

tp::App::App()
	: m_window(
		  sf::VideoMode::getDesktopMode().width,
		  sf::VideoMode::getDesktopMode().height,
		  "High-quality paths amid polygonal obstacles")
	, m_voronoi(m_window.GetBounds())
{
	m_gui.onInit(&m_window);

	m_window.SetMouseCallback([this](int button, int x, int y) {
		if (!ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow) && !ImGui::IsAnyItemHovered())
		{
			OnMouseClick(button, x, y);
		}
	});
	m_window.SetKeyboardCallback([this](keyboard::Key key) { OnKeyPressed(key); });

	m_voronoi.SetBorderRect(GetAdjustedWindowSize(m_window, m_state));
}

void tp::App::Run()
{
	using namespace tp;
	using namespace tp::shapes;

	sf::Clock clock;
	while (m_window.IsRunning())
	{
		sf::Event event;
		while (m_window.GetRaw()->pollEvent(event))
		{
			m_gui.HandleEvent(event);
			m_window.HandleEvent(event);
		}
		m_gui.onUpdate(&m_window, clock);

		m_gui.onRender(&m_window, m_state, [this]() {
			RefreshAll(m_state, m_pathFinder, m_voronoi);
		});

		m_window.Clear(m_state.backgroundColor);

		if (m_state.showStartEndPoints)
		{
			m_window.Draw(m_state.start, Color::Magenta);
			m_window.Draw(m_state.end, Color::Magenta);
		}

		if (m_state.showVoronoiCells)
		{
			for (const auto& cell : m_state.voronoiData.all_tilde_V_cells)
			{
				m_window.Draw(cell, Color(0, 255, 0, 64));
			}
		}

		if (m_state.showVoronoiEdges)
		{
			for (const auto& edge : m_state.voronoiData.refined_edges)
			{
				m_window.Draw(edge, edge.color, 2.f);
			}
		}

		if (m_state.showPathFindingEdges)
		{
			for (const auto& seg : m_state.edges)
			{
				m_window.Draw(seg, Color::Magenta);
			}
		}

		if (m_state.showPath)
		{
			for (const auto& seg : m_state.path)
			{
				m_window.Draw(seg, m_state.pathColor, 3.f);
			}
		}

		if (m_state.showObstacles)
		{
			for (size_t i = 0; i < m_state.obstacles.size(); ++i)
			{
				Color drawColor = i == m_state.selectedObstacleIndex ? m_state.selectedObstacleColor : m_state.obstacleColor;
				m_window.Draw(m_state.obstacles[i], drawColor);
			}
		}

		if (m_state.showStartEndCells)
		{
			if (m_state.voronoiData.s_cell_idx != -1)
			{
				m_window.Draw(m_state.voronoiData.all_tilde_V_cells[m_state.voronoiData.s_cell_idx], Color(0, 255, 0, 64));
			}
			if (m_state.voronoiData.t_cell_idx != -1)
			{
				m_window.Draw(m_state.voronoiData.all_tilde_V_cells[m_state.voronoiData.t_cell_idx], Color(0, 255, 0, 64));
			}
		}

		if (m_state.isBuilding && !m_state.currentObstacle.empty())
		{
			if (m_state.currentObstacle.size() == 1)
			{
				m_window.Draw(m_state.currentObstacle[0], m_state.buildingColor);
			}
			else
			{
				if (m_state.currentObstacle.size() == 1)
				{
					m_window.Draw(m_state.currentObstacle.front(), m_state.buildingColor);
				}
				else if (m_state.currentObstacle.size() == 2)
				{
					m_window.Draw(Segment{ m_state.currentObstacle[0], m_state.currentObstacle[1] }, m_state.buildingColor);
				}
				else
				{
					m_window.Draw(m_state.currentObstacle, m_state.buildingColor);
				}
			}
		}

		// DRAW ON WINDOW

		m_gui.onDisplay(&m_window);
		m_window.Display();
	}
}

void tp::App::OnMouseClick(int button, int x, int y)
{
	Point clickPos = { static_cast<float>(x), static_cast<float>(y) };
	if (clickPos.x > GetAdjustedWindowSize(m_window, m_state).right)
		return;

	if (button == 2)
	{
		m_state.start = clickPos;
		RefreshAll(m_state, m_pathFinder, m_voronoi);
	}
	if (button == 3)
	{
		m_state.end = clickPos;
		RefreshAll(m_state, m_pathFinder, m_voronoi);
	}
	if (button == 0)
	{
		m_state.hoveredObstacleIndex = -1;

		if (ImGui::GetIO().WantCaptureMouse)
			return;

		for (size_t i = 0; i < m_state.obstacles.size(); ++i)
		{
			if (math::Contains(clickPos, m_state.obstacles[i]))
			{
				m_state.hoveredObstacleIndex = static_cast<int>(i);
				break;
			}
		}

		if (m_state.hoveredObstacleIndex != -1 && !m_state.isBuilding)
		{

			m_state.selectedObstacleIndex = m_state.hoveredObstacleIndex;
			LOG_DEBUG(std::format("Selected obstacle {}", m_state.selectedObstacleIndex));
		}
		else if (!m_state.isBuilding)
		{
			m_state.selectedObstacleIndex = -1;
			m_state.isBuilding = true;
			m_state.currentObstacle.clear();
			LOG_DEBUG(std::format("Started building obstacle {}", m_state.obstacles.size()));
			m_state.currentObstacle.push_back(clickPos);
			LOG_DEBUG(std::format("Added Point: [{}, {}]", clickPos.x, clickPos.y));
		}
		else
		{
			m_state.currentObstacle.push_back(clickPos);
			LOG_DEBUG(std::format("Added Point: [{}, {}]", clickPos.x, clickPos.y));
		}
	}
	if (button == 1)
	{
		if (m_state.isBuilding)
		{
			if (!m_state.currentObstacle.empty())
			{
				if (m_state.currentObstacle.size() < 3)
				{
					LOG_DEBUG("Obstacle needs at least 3 points, clearing.");
					return m_state.currentObstacle.clear();
				}
				m_state.obstacles.push_back(m_state.currentObstacle);
				LOG_DEBUG(std::format("Obstacle created with {} points", m_state.currentObstacle.size()));

				RefreshAll(m_state, m_pathFinder, m_voronoi);
			}

			m_state.currentObstacle.clear();
		}
		else
		{
			m_state.selectedObstacleIndex = -1;
			LOG_DEBUG("Deselected obstacle.");
		}
		m_state.isBuilding = false;
	}
}

void tp::App::OnKeyPressed(keyboard::Key key)
{
	if (key == keyboard::Key::Escape)
	{
		m_window.Close();
	}
	if (key == keyboard::Key::Space)
	{
		RefreshAll(m_state, m_pathFinder, m_voronoi);
	}
	if (key == keyboard::Key::F1)
	{
		m_state.currGraphType = m_state.currGraphType == GraphType::G1 ? GraphType::G2 : GraphType::G1;
		GeneratePath(m_state, m_pathFinder);
	}
}
