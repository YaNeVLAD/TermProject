#include "GraphUtils.h"

#include <algorithm>
#include <iostream>

std::vector<unsigned> tp::ShortestHamiltonianPath(const Matrix& matrix, const std::string& input)
{
	std::vector<unsigned> path(matrix.GetSize());
	for (size_t i = 0; i < path.size(); ++i)
	{
		path[i] = static_cast<unsigned>(i);
	}

	try
	{
		size_t bestLen = ULLONG_MAX;
		std::vector<unsigned> bestCycle;

		std::sort(path.begin(), path.end());
		do
		{
			std::vector<unsigned> cycle(path);
			cycle.push_back(cycle.front());

			auto len = matrix.CalculatePathLength(cycle);
			if (len != 0 && len < bestLen)
			{
				bestCycle = cycle;
				bestLen = len;
			}
		} while (std::next_permutation(path.begin(), path.end()));

		return bestCycle;
	}
	catch (const std::exception& ex)
	{
		std::cout << "Error: " << ex.what() << std::endl;
	}
}
