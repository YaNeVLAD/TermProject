#include "UserInterface.h"

#include <format>
#include <functional>

#include "imgui-SFML.h"
#include "imgui.h"
#include <SFML/Graphics.hpp>

#include "../../Utility/Loader/PolygonLoader.h"
#include "../../Utility/Logger/Logger.h"
#include "../Window.h"

void tp::UserInterface::onInit(Window* window) const
{
	if (window)
	{
		ImGui::SFML::Init(*window->GetRaw());
	}
}

void tp::UserInterface::onUpdate(Window* window, sf::Clock& clock) const
{
	ImGui::SFML::Update(*window->GetRaw(), clock.restart());
}

void tp::UserInterface::onDisplay(Window* window) const
{
	if (window)
	{
		ImGui::SFML::Render(*window->GetRaw());
	}
}

void tp::UserInterface::onRender(Window* window, AppState& state, std::function<void()> onObstacleEditCallback) const
{
	ImVec2 main_window_size = ImGui::GetIO().DisplaySize; // This gets the actual rendering surface size (SFML window size)

	ImVec2 window_pos = ImVec2(main_window_size.x - state.settingsWindowWidth, 0);
	ImVec2 window_size = ImVec2(state.settingsWindowWidth, main_window_size.y);

	ImGui::SetNextWindowPos(window_pos);
	ImGui::SetNextWindowSize(window_size);

	// --- Window Flags ---
	ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoMove
		| ImGuiWindowFlags_NoResize
		| ImGuiWindowFlags_NoCollapse
		| ImGuiWindowFlags_NoSavedSettings;

	ImGui::Begin("Settings", nullptr, window_flags);

	// --- Tab Bar ---
	if (ImGui::BeginTabBar("MyTabBar"))
	{
		// --- General Settings Tab ---
		if (ImGui::BeginTabItem("General"))
		{
			// --- Color Editing Section ---
			if (ImGui::CollapsingHeader("Colors"))
			{
				const char* colorNames[] = {
					"Edge Color",
					"Path Color",
					"Obstacle Color",
					"Building Color",
					"Background Color",
					"Selected Obstacle Color",
				};
				static int currentColorIndex = 0;

				ImGui::Combo("Select Color", &currentColorIndex, colorNames, IM_ARRAYSIZE(colorNames));

				Color* selectedColor = nullptr;
				switch (currentColorIndex)
				{
				case 0:
					selectedColor = &state.edgeColor;
					break;
				case 1:
					selectedColor = &state.pathColor;
					break;
				case 2:
					selectedColor = &state.obstacleColor;
					break;
				case 3:
					selectedColor = &state.buildingColor;
					break;
				case 4:
					selectedColor = &state.backgroundColor;
					break;
				case 5:
					selectedColor = &state.selectedObstacleColor;
					break;
				}

				if (selectedColor)
				{
					float color[4] = {
						selectedColor->r / 255.0f,
						selectedColor->g / 255.0f,
						selectedColor->b / 255.0f,
						selectedColor->a / 255.0f
					};

					if (ImGui::ColorEdit4(colorNames[currentColorIndex], color))
					{
						selectedColor->r = static_cast<uint8_t>(color[0] * 255.0f);
						selectedColor->g = static_cast<uint8_t>(color[1] * 255.0f);
						selectedColor->b = static_cast<uint8_t>(color[2] * 255.0f);
						selectedColor->a = static_cast<uint8_t>(color[3] * 255.0f);
					}
				}
			}

			// --- Display Options Section ---
			if (ImGui::CollapsingHeader("Display Options"))
			{
				ImGui::Checkbox("Show Start/End Points", &state.showStartEndPoints);
				ImGui::Checkbox("Show Voronoi Cells", &state.showVoronoiCells);
				ImGui::Checkbox("Show Voronoi Edges", &state.showVoronoiEdges);
				ImGui::Checkbox("Show Path", &state.showPath);
				ImGui::Checkbox("Show Pathfinding Edges", &state.showPathFindingEdges);
				ImGui::Checkbox("Show Obstacles", &state.showObstacles);
				ImGui::Checkbox("Show Start/End Cells", &state.showStartEndCells);
			}

			// --- Voronoi & Pathfinding Parameters Section ---
			if (ImGui::CollapsingHeader("Algorithm Parameters"))
			{
				int segments = static_cast<int>(state.segmentsPerArc);
				if (ImGui::SliderInt("Segments Per Arc", &segments, 1, 5))
				{
					state.segmentsPerArc = static_cast<size_t>(segments);
				}
				ImGui::Text(" (Affects Voronoi re-generation)");

				const char* graphTypes[] = { "G1", "G2" };
				int currentGraphTypeIndex = static_cast<int>(state.currGraphType);

				if (ImGui::Combo("Graph Type", &currentGraphTypeIndex, graphTypes, IM_ARRAYSIZE(graphTypes)))
				{
					state.currGraphType = static_cast<GraphType>(currentGraphTypeIndex);
				}
				ImGui::Text(" (Affects Pathfinding re-calculation)");
			}

			if (ImGui::Button("Load scene from file"))
			{
				state.polygonLoadError.clear();
				std::string selectedPath = tp::loader::OpenFileViaIFileOpenDialog();

				if (!selectedPath.empty())
				{
					state.polygonLoadFilename = selectedPath;
					std::vector<tp::shapes::Polygon> loadedPolygons = tp::loader::LoadPolygonsFromFile(state.polygonLoadFilename);

					if (loadedPolygons.empty())
					{
						state.polygonLoadError = "Failed to load polygons or file was empty.";
					}
					else
					{
						state.obstacles = loadedPolygons;
						state.selectedObstacleIndex = -1;
						state.hoveredObstacleIndex = -1;
						onObstacleEditCallback();
						LOG_DEBUG(std::format("Loaded {} polygons from {}", loadedPolygons.size(), state.polygonLoadFilename));
					}
				}
				else
				{
					state.polygonLoadError = "File dialog cancelled or failed.";
				}
			}

			if (!state.polygonLoadError.empty())
			{
				ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "Error: %s", state.polygonLoadError.c_str());
			}

			ImGui::EndTabItem();
		} // End General Tab

		// --- Obstacle Editing Tab ---
		if (ImGui::BeginTabItem("Obstacles"))
		{
			if (ImGui::CollapsingHeader("Obstacle List"))
			{
				if (state.obstacles.empty())
				{
					ImGui::Text("No obstacles created yet.");
				}
				else
				{
					for (size_t i = 0; i < state.obstacles.size(); ++i)
					{
						char label[64];
						sprintf_s(label, "Obstacle %zu", i);
						if (ImGui::Selectable(label, state.selectedObstacleIndex == i))
						{
							state.selectedObstacleIndex = (state.selectedObstacleIndex == i) ? -1 : static_cast<int>(i);
						}
					}

					ImGui::Separator();
					if (state.selectedObstacleIndex != -1)
					{
						if (ImGui::Button("Delete Selected Obstacle"))
						{
							if (state.selectedObstacleIndex < state.obstacles.size())
							{
								state.obstacles.erase(state.obstacles.begin() + state.selectedObstacleIndex);
								state.selectedObstacleIndex = -1;
								onObstacleEditCallback();
							}
						}
					}
				}
			}

			if (state.selectedObstacleIndex != -1 && state.selectedObstacleIndex < state.obstacles.size())
			{
				ImGui::Separator();
				if (ImGui::CollapsingHeader(std::format("Edit Obstacle {}", state.selectedObstacleIndex).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
				{
					tp::shapes::Polygon& currentEditObstacle = state.obstacles[state.selectedObstacleIndex];

					ImGui::Text("Points:");
					for (size_t i = 0; i < currentEditObstacle.size(); ++i)
					{
						float coords[2] = { currentEditObstacle[i].x, currentEditObstacle[i].y };
						ImGui::PushID(static_cast<int>(i));
						if (ImGui::InputFloat2("##Point", coords))
						{
							currentEditObstacle[i].x = coords[0];
							currentEditObstacle[i].y = coords[1];
							onObstacleEditCallback();
						}
						ImGui::PopID();
					}

					if (ImGui::Button("Remove Last Point"))
					{
						if (!currentEditObstacle.empty())
						{
							if (currentEditObstacle.size() > 3)
							{
								currentEditObstacle.pop_back();
							}
							onObstacleEditCallback();
						}
					}
				}
			}
			ImGui::EndTabItem();
		} // End Obstacles Tab
		ImGui::EndTabBar();
	} // End TabBar

	ImGui::End();
}

void tp::UserInterface::onDestroy(Window* window) const
{
	if (window)
	{
		ImGui::SFML::Shutdown();
	}
}

void tp::UserInterface::HandleEvent(const sf::Event& event) const
{
	ImGui::SFML::ProcessEvent(event);
}
