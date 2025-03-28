#include "Matrix.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

unsigned& tp::Matrix::operator()(size_t row, size_t col)
{
	if (row >= m_rows || col >= m_cols)
	{
		throw std::out_of_range("Matrix index out of bounds");
	}

	return m_data[row][col];
}

const unsigned& tp::Matrix::operator()(size_t row, size_t col) const
{
	if (row >= m_rows || col >= m_cols)
	{
		throw std::out_of_range("Matrix index out of bounds");
	}
	return m_data[row][col];
}

std::vector<unsigned>& tp::Matrix::operator[](size_t row)
{
	if (row >= m_rows)
	{
		throw std::out_of_range("Matrix index out of bounds");
	}
	return m_data[row];
}

size_t tp::Matrix::CalculatePathLength(const std::vector<unsigned>& path) const
{
	if (path.empty())
	{
		return 0;
	}

	size_t len = 0;
	for (size_t i = 0; i < path.size() - 1; ++i)
	{
		size_t u = path[i];
		size_t v = path[i + 1];

		len += m_data[u][v];
	}

	return len;
}

tp::Matrix tp::Matrix::CreateFromFile(const std::string& name)
{
	std::ifstream input(name);
	if (!input.is_open())
	{
		throw std::runtime_error("Failed to open file");
	}

	std::string line;
	std::getline(input, line);

	line.erase(std::remove_if(line.begin(), line.end(), ::isspace), line.end());
	size_t xPos = line.find('x');
	if (xPos == std::string::npos)
	{
		xPos = line.find('X');
		if (xPos == std::string::npos)
		{
			throw std::runtime_error("Invalid format: missing 'x' or 'X' in dimensions");
		}
	}

	size_t rows = std::stoull(line.substr(0, xPos));
	size_t cols = std::stoull(line.substr(xPos + 1));

	if (rows <= 0 || cols <= 0)
	{
		throw std::runtime_error("Invalid dimensions: rows and columns must be greater than 0");
	}

	Matrix matrix(rows, cols);

	for (size_t i = 0; i < rows; ++i)
	{
		std::getline(input, line);
		std::istringstream iss(line);
		for (size_t j = 0; j < cols; ++j)
		{
			iss >> matrix(i, j);
		}
	}

	return matrix;
}
