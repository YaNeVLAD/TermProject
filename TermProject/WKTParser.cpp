#include "WKTParser.h"

#include <fstream>
#include <sstream>

namespace path_finder
{

Point WKTParser::WKTVisitor::ParsePoint(std::istream& input)
{
	char ch;
	double x, y;
	input >> ch >> x >> y;
	return Point(x, y);
}

Obstacle WKTParser::WKTVisitor::ParseObstacle(std::istream& input)
{
	std::vector<Point> points;
	char ch;
	double x, y;

	input >> ch;
	while (input >> x >> y)
	{
		points.emplace_back(x, y);
		input >> ch;
		if (ch == ')')
		{
			break;
		}
	}

	return Obstacle(points);
}

/**
 * \throws std::runtime_error
 */
void WKTParser::Parse(const std::string& fileName)
{
	std::ifstream input(fileName);
	if (!input.is_open())
	{
		throw std::runtime_error("Failed to open file " + fileName);
	}

	std::string line;
	std::string type;
	while (std::getline(input, line))
	{
		std::istringstream iss(line);
		if (iss >> type; type == "POINT")
		{
			Point p = m_Visitor.ParsePoint(iss);
		}
		else if (type == "POLYGON")
		{
			Obstacle o = m_Visitor.ParseObstacle(iss);
		}
	}
}

} // namespace path_finder
