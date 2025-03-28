#include "ShapeBuilder.h"
#include <fstream>
#include <sstream>
#include <stdexcept>

using namespace std::literals;

namespace WKT
{
constexpr std::string_view POLYGON = "POLYGON"sv;
constexpr char CORDS_START = '(';
constexpr char CORDS_END = ')';
constexpr char CORDS_SEPARATOR = ',';
} // namespace WKT

static std::string Trim(const std::string& str)
{
	size_t first = str.find_first_not_of(' ');
	if (first == std::string::npos)
		return "";
	size_t last = str.find_last_not_of(' ');
	return str.substr(first, last - first + 1);
}

std::vector<tp::Shapes::Polygon> tp::ShapeBuilder::BuildFromWkt(const std::string& name)
{
	using namespace tp::Shapes;

	std::ifstream input(name);
	if (!input.is_open())
	{
		throw std::runtime_error("Failed to open file" + name);
	}

	std::vector<Polygon> result;

	std::string line;
	while (std::getline(input, line))
	{
		try
		{
			result.push_back(FromWktString(line));
		}
		catch (const std::exception&)
		{
			continue;
		}
	}

	return result;
}

tp::Shapes::Polygon tp::ShapeBuilder::FromWktString(const std::string& str)
{
	using namespace tp::Shapes;

	Polygon polygon;
	std::stringstream ss(str);
	std::string token;

	std::getline(ss, token, WKT::CORDS_START);
	if (Trim(token) != WKT::POLYGON)
	{
		throw std::runtime_error("Invalid WKT format: Expected POLYGON");
	}

	std::getline(ss, token, WKT::CORDS_END);
	std::stringstream coordStream(token);

	std::getline(coordStream, token, WKT::CORDS_END);
	token = Trim(token);
	if (token.front() == WKT::CORDS_START)
	{
		token.erase(0, 1);
	}
	if (token.back() == WKT::CORDS_END)
	{
		token.pop_back();
	}

	std::stringstream pointsStream(token);
	std::string pointToken;
	while (std::getline(pointsStream, pointToken, WKT::CORDS_SEPARATOR))
	{
		pointToken = Trim(pointToken);
		std::stringstream pointStream(pointToken);
		Point point;
		pointStream >> point.x >> point.y;
		polygon.push_back(point);
	}

	return polygon;
}
