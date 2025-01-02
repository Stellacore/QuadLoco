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
\brief Unit tests (and example) code for quadloco::ops::SymRing
*/


#include "imgChipSpec.hpp"
#include "io.hpp"
#include "opsSymRing.hpp"
#include "prbStats.hpp"
#include "rasgrid.hpp"
#include "rasGrid.hpp"

#include <iostream>
#include <sstream>


namespace
{
	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		using namespace quadloco;

		// create grid and populate with a square annular signal
		// annular square should have point symmetry response for
		// symmetry filter radius greater than half square size
		// (and less than half diagonal).
		ras::SizeHW const fullSize{ 30u, 50u };
		ras::Grid<float> fGrid(fullSize);
		img::ChipSpec const smlChip
			{ ras::RowCol{ 12u, 22u }
			, ras::SizeHW{  7u,  9u }
			};
		img::ChipSpec const bigChip
			{ ras::RowCol{ 10u, 20u }
			, ras::SizeHW{ 11u, 13u }
			};
		ras::RowCol const expPeakRC{ 10u+5u, 20u+6u };
		//
		std::fill(fGrid.begin(), fGrid.end(), 0.f);
		ras::grid::setSubGridValues(&fGrid, bigChip, -1.f);
		ras::grid::setSubGridValues(&fGrid, smlChip,  1.f);

		// technique 1 - create filter first then apply

		// create symmetry filter - reusing available source stats
		prb::Stats<float> const fStats(fGrid.cbegin(), fGrid.cend());
		std::size_t const halfSize{ 4u }; // a bit larger than box half side
		ops::SymRing const symRing(&fGrid, fStats, halfSize);

		// apply symmetry filter
		ras::PeakRCV peakRCV; // track (first occuring) maximum in response
		ras::Grid<float> const symGrid1
			{ ops::symRingGridFor(fGrid, symRing, &peakRCV) };

		// technique 2 - get filter result directly

		// get response grid (function generates needed stats)
		// (also could have set peakRCV)
		ras::Grid<float> const symGrid2{ ops::symRingGridFor(fGrid, halfSize) };

		// [DoxyExample01]

		ras::RowCol const gotPeakRC{ peakRCV.theRow, peakRCV.theCol };
		if (! (gotPeakRC == expPeakRC))
		{
			oss << "Failiure of gotPeakRC test\n";
			oss << "exp: " << expPeakRC << '\n';
			oss << "got: " << gotPeakRC << '\n';
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

