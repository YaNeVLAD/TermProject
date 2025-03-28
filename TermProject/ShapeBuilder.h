#pragma once

#include <istream>
#include <string>

#include "Shapes.h"

namespace tp::ShapeBuilder
{

/**
 * \throws std::runtime_error - if failed to open file
 */
std::vector<tp::Shapes::Polygon> BuildFromWkt(const std::string& name);

tp::Shapes::Polygon FromWktString(const std::string& str);
} // namespace tp::ShapeBuilder
