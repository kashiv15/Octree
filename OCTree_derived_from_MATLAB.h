#pragma once
#include <iostream> 
#include <fstream>
#include <vector> 
#include <Eigen\Dense>
#include <math.h>

using namespace std;

// Template which produces a vector of equally space points from "start" to "end" by "num" many points
template <typename T> std::vector<T> linspace(int start, int end, int num)
{
	std::vector<T> linspaced;

	if (0 != num)
	{
		if (1 == num)
		{
			linspaced.push_back(static_cast<T>(start));
		}
		else
		{
			double delta = (end - start) / (num - 1);

			for (auto i = 0; i < (num - 1); ++i)
			{
				linspaced.push_back(static_cast<T>(start + delta * i));
			}
			// ensure that start and end are exactly the same as the input
			linspaced.push_back(static_cast<T>(end));
		}
	}
	return linspaced;
}