#ifndef APP_STATE_H
#define APP_STATE_H

#include <string>
#include <vector>

#include "../Algorithm/PathFinder/PathFinder.h"
#include "../Algorithm/Voronoi/Voronoi.h"
#include "../Utility/Color/Color.h"
#include "../Utility/Shapes.hpp"

namespace tp
{
class AppState
{
public:
	// Obstacle management
	shapes::Polygon currentObstacle;
	std::vector<shapes::Polygon> obstacles;
	bool isBuilding = false;

	size_t selectedObstacleIndex = -1;
	size_t hoveredObstacleIndex = -1;

	// Voronoi and Pathfinding data
	VoronoiData voronoiData;
	std::vector<shapes::Segment> path;
	std::vector<shapes::Segment> edges;

	// Graph type
	size_t segmentsPerArc = 2;
	GraphType currGraphType = GraphType::G1;

	// Start and End points
	shapes::Point start = { 100, 500 };
	shapes::Point end = { 1000, 500 };

	// Colors
	Color edgeColor = Color{ 0x246AF3FF };
	Color pathColor = Color::Green;
	Color obstacleColor = Color{ 0xFF603DFF };
	Color buildingColor = Color::Green;
	Color backgroundColor = Color::White;
	Color selectedObstacleColor = Color{ 0xFF00FF80 };
	Color hoveredObstacleColor = Color{ 0x00FFFFFF };

	// Flags
	bool showStartEndPoints = true;
	bool showVoronoiCells = false;
	bool showVoronoiEdges = true;
	bool showPath = true;
	bool showPathFindingEdges = false;
	bool showObstacles = true;
	bool showStartEndCells = false;

	float settingsWindowWidth = 500.f;

	std::string polygonLoadFilename;
	std::string polygonLoadError;
};
} // namespace tp

#endif
