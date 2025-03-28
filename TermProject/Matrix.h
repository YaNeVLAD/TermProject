#pragma once

#include <string>
#include <vector>

namespace tp
{
class Matrix
{
	size_t m_rows;
	size_t m_cols;
	std::vector<std::vector<unsigned>> m_data;

public:
	Matrix(size_t rows, size_t columns)
		: m_rows(rows)
		, m_cols(columns)
		, m_data(rows, std::vector<unsigned>(columns))
	{
	}

	Matrix(std::vector<std::vector<unsigned>> data)
		: m_rows(data.at(0).size())
		, m_cols(data.size())
		, m_data(data)
	{
	}

	unsigned& operator()(size_t row, size_t col);

	const unsigned& operator()(size_t row, size_t col) const;

	std::vector<unsigned>& operator[](size_t row);

	size_t CalculatePathLength(const std::vector<unsigned>& path) const;

	static Matrix CreateFromFile(const std::string& name);

	size_t GetRowsCount() const { return m_rows; }
	size_t GetColumnsCount() const { return m_cols; }
	size_t GetSize() const { return std::max(m_rows, m_cols); }
};

} // namespace tp
