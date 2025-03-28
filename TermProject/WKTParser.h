#pragma once

#include "PathFinder.h"
#include <istream>

namespace path_finder
{
class WKTParser
{
	class WKTVisitor
	{
	public:
		WKTVisitor() = default;

		Point ParsePoint(std::istream& input);
		Obstacle ParseObstacle(std::istream& input);
	};

public:
	void Parse(const std::string& fileName);

private:
	WKTVisitor m_Visitor;
};

} // namespace path_finder
