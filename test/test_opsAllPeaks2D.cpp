//
// MIT License
//
// Copyright (c) 2024 Stellacore Corporation
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//


/*! \file
\brief Unit tests (and example) code for quadloco::ops::AllPeaks2D
*/


#include "QuadLoco/opsAllPeaks2D.hpp"
#include "QuadLoco/rasGrid.hpp"
#include "QuadLoco/rasPeakRCV.hpp"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <vector>


namespace
{
	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		using namespace quadloco;

		ras::Grid<double> dataGrid(50u, 100u);

		// define a bunch of known peaks
		// NOTE: edge of grid is not considered (e.g. only cells with
		//       that have a full 8-hood around them are considered!
		std::vector<ras::PeakRCV> expPeakRCVs
			// - near grid boundaries (one in)
			{ { ras::RowCol{  1,  1 }, 1. }
			, { ras::RowCol{  1, 98 }, 1. }
			, { ras::RowCol{ 48,  1 }, 1. }
			, { ras::RowCol{ 48, 98 }, 1. }
			// - edge adjacent
			, { ras::RowCol{  3,  1 }, 2. }
			, { ras::RowCol{  3,  2 }, 2. }
			//
			, { ras::RowCol{  5,  5 }, 3. }
			, { ras::RowCol{  6,  5 }, 3. }
			// - diagonal adjacent
			, { ras::RowCol{ 10, 10 }, 4. }
			, { ras::RowCol{ 11, 11 }, 4. }
			//
			, { ras::RowCol{ 19, 20 }, 5. }
			, { ras::RowCol{ 20, 19 }, 5. }
			};

		// smooth grid with gaussian then run again

		// [DoxyExample01]

		std::fill(dataGrid.begin(), dataGrid.end(), 0.);
		for (ras::PeakRCV const & expPeakRCV : expPeakRCVs)
		{
			dataGrid(expPeakRCV.theRowCol) = expPeakRCV.theValue;
		}

		// find filter response peaks sorted in order of peak value
		ops::AllPeaks2D const allPeaks(dataGrid);
		std::vector<ras::PeakRCV> const gotPeakRCVs
			{ allPeaks.largestPeakRCVs() };

		// [DoxyExample01]

		bool badTest{ false };
		std::vector<ras::PeakRCV> difPeakRCVs{};

		if (! (gotPeakRCVs.size() == expPeakRCVs.size()))
		{
			oss << "Failure of gotPeakRCVs.size() test\n";
			badTest = true;
		}
		else
		{
			// expect peaks in order largest value to smallest
			std::sort(expPeakRCVs.rbegin(), expPeakRCVs.rend());

			// compare individual peaks
			difPeakRCVs.reserve
				(std::max(expPeakRCVs.size(), gotPeakRCVs.size()));
			std::set_difference
				( gotPeakRCVs.begin(), gotPeakRCVs.end()
				, expPeakRCVs.begin(), expPeakRCVs.end()
				, std::inserter(difPeakRCVs, difPeakRCVs.end())
				);
			if (! difPeakRCVs.empty())
			{
				oss << "Failure of difPeakRCVs test\n";
				badTest = true;
			}
		}

		if (badTest)
		{
			oss << "expPeakRCVs\n";
			for (ras::PeakRCV const & expPeakRCV : expPeakRCVs)
			{
				oss << "  " << expPeakRCV << '\n';
			}

			oss << "gotPeakRCVs\n";
			for (ras::PeakRCV const & gotPeakRCV : gotPeakRCVs)
			{
				oss << "  " << gotPeakRCV << '\n';
			}

			oss << "difPeakRCVs\n";
			for (ras::PeakRCV const & difPeakRCV : difPeakRCVs)
			{
				oss << "  " << difPeakRCV << '\n';
			}
		}

	}

}

//! Standard test case main wrapper
int
main
	()
{
	int status{ 1 };
	std::stringstream oss;

//	test0(oss);
	test1(oss);

	if (oss.str().empty()) // Only pass if no errors were encountered
	{
		status = 0;
	}
	else
	{
		// else report error messages
		std::cerr << "### FAILURE in test file: " << __FILE__ << std::endl;
		std::cerr << oss.str();
	}
	return status;
}

