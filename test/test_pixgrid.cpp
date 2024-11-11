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
\brief Unit tests (and example) code for quadloco::pix::grid:: functions
*/


#include "pixgrid.hpp"

#include "datChipSpec.hpp"
#include "datGrid.hpp"
#include "pixGradel.hpp"
#include "pixgrid.hpp"

#include <algorithm>
#include <format>
#include <iostream>
#include <sstream>



namespace
{
	//! Test border filling functions
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample00]

		// define arbitrary grid
		quadloco::dat::Grid<char> grid(10u, 7u);
		std::fill(grid.begin(), grid.end(), '.');

		// fill outer two rows and outer two columns
		// This uses a slightly faster algorithm than the individual
		// fill operations below
		quadloco::pix::grid::fillBorder
			(grid.begin(), grid.hwSize(), 2u, 'x');

		// fill outer one cell thick perimeter - one edge at a time
		quadloco::pix::grid::fillInitRows
			(grid.begin(), grid.hwSize(), 1u, 'W');
		quadloco::pix::grid::fillInitCols
			(grid.begin(), grid.hwSize(), 1u, 'W');
		quadloco::pix::grid::fillLastCols
			(grid.begin(), grid.hwSize(), 1u, 'W');
		quadloco::pix::grid::fillLastRows
			(grid.begin(), grid.hwSize(), 1u, 'W');

		std::size_t const gotNumW // number of set 1-cell outer border
			{ (std::size_t)std::count(grid.cbegin(), grid.cend(), 'W') };
		std::size_t const gotNumX // number of remaining 2-cell border
			{ (std::size_t)std::count(grid.cbegin(), grid.cend(), 'x') };

		// [DoxyExample00]


		using namespace quadloco;

		std::size_t const expNumW
			{ 2u * grid.wide() // top and bottom
			+ 2u * grid.high() // left and right
			- 4u  // double counts
			};
		std::size_t const expNumX
			{ 2u * (grid.wide()-2u) // top and bottom
			+ 2u * (grid.high()-2u) // left and right
			- 4u  // double counts
			};

		if (! (gotNumW == expNumW))
		{
			oss << "Failure of count NumW-cell edge test(0)\n";
			oss << "exp: " << expNumW << '\n';
			oss << "got: " << gotNumW << '\n';
			oss << grid.infoStringContents("grid", "%c") << '\n';
		}

		if (! (gotNumX == expNumX))
		{
			oss << "Failure of count NumX-cell edge test(0)\n";
			oss << "exp: " << expNumX << '\n';
			oss << "got: " << gotNumX << '\n';
			oss << grid.infoStringContents("grid", "%c") << '\n';
		}

	}

	//! Test simple edge detection
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		// using tiny data grids for easy test data inspection
		constexpr std::size_t ndxHalf{ 4u };
		constexpr std::size_t ndxFull{ 2u*ndxHalf };
		quadloco::dat::SizeHW const hwSize{ ndxFull, ndxFull };
		// one with vertical, one with horizontal edges
		quadloco::dat::Grid<float> tbPixels(hwSize); // Top-to-Bot edge
		quadloco::dat::Grid<float> lrPixels(hwSize); // Lft-to-Rgt edge
		constexpr float backVal{ -15.25f };
		constexpr float foreVal{  -5.25f };
		// initialize both grids to background values
		std::fill(tbPixels.begin(), tbPixels.end(), backVal);
		std::fill(lrPixels.begin(), lrPixels.end(), backVal);

		// Step sizes used to evaluate gradient for this test
		constexpr std::size_t stepHalf{ 1u };
		constexpr std::size_t stepFull{ 2u * stepHalf };

		// set foreground for bottom half of the horizontal edge grid
		std::fill(tbPixels.beginRow(ndxHalf), tbPixels.end(), foreVal);
		quadloco::pix::Gradel const tbExpGradel{ 10./(double)stepFull, 0. };

		// Use ChipSpec to set right half foreground for vertical edge grid
		quadloco::dat::ChipSpec const lrFillSpec
			{ quadloco::dat::RowCol{ 0u, lrPixels.wide()/2u }
			, quadloco::dat::SizeHW{ lrPixels.high(), lrPixels.wide()/2u }
			};
		quadloco::pix::grid::setSubGridValues(&lrPixels, lrFillSpec, foreVal);
		quadloco::pix::Gradel const lrExpGradel{ 0., 10./(double)stepFull };

		// Compute edge gradient across stepFull pixels

		// gradient magnitude prop to:
		// [gridVal(rc + stepHalf) - gridVal(rc - stepHalf)] / (2*stepHalf)
		// Vertical grid gradients
		quadloco::dat::Grid<quadloco::pix::Gradel> const lrGradels
			{ quadloco::pix::grid::gradelGridFor(lrPixels, stepHalf) };
		// Horizontal grid gradients
		quadloco::dat::Grid<quadloco::pix::Gradel> const tbGradels
			{ quadloco::pix::grid::gradelGridFor(tbPixels, stepHalf) };

		// [DoxyExample01]


		using namespace quadloco::dat;
		using namespace quadloco::pix;

		// Extract computed gradient values within edge regions
		ChipSpec const tbChipSpec{ RowCol{ 3u, 1u }, SizeHW{ 2u, 6u } };
		Grid<Gradel> const tbGotChipGels
			{ grid::subGridValuesFrom(tbGradels, tbChipSpec) };
		//
		ChipSpec const lrChipSpec{ RowCol{ 1u, 3u }, SizeHW{ 6u, 2u } };
		Grid<Gradel> const lrGotChipGels
			{ grid::subGridValuesFrom(lrGradels, lrChipSpec) };

		// Create expected chips populated with expected gradel values
		Grid<Gradel> tbExpChipGels(tbGotChipGels.hwSize());
		std::fill(tbExpChipGels.begin(), tbExpChipGels.end(), tbExpGradel);
		//
		Grid<Gradel> lrExpChipGels(lrGotChipGels.hwSize());
		std::fill(lrExpChipGels.begin(), lrExpChipGels.end(), lrExpGradel);

		// Check if extracted edge values match expected ones

		std::function<std::string(Gradel const &)> const fmtFunc
			{ [] (Gradel const & elem)
				{ return std::format("({:4.1f},{:4.1f})", elem[0], elem[1]); }
			};

		std::function<bool(Gradel const & gdelA, Gradel const & gdelB)>
			const nearlyEqualGradels
			{ [] (Gradel const & gdelA, Gradel const & gdelB)
				{ return nearlyEquals(gdelA, gdelB); }
			};

		bool const tbOkay
			{ (tbGotChipGels.hwSize() == tbExpChipGels.hwSize())
			&& std::equal
				( tbGotChipGels.cbegin(), tbGotChipGels.cend()
				, tbExpChipGels.cbegin()
				, nearlyEqualGradels
				)
			};

		if (! tbOkay)
		{
			oss << "Failure of tb*ChipGels test\n";
			oss << tbPixels
				.infoStringContents("tbPixels", "%5.0f") << '\n';
			oss << tbGradels
				.infoStringContents("tbGradels", fmtFunc) << '\n';
			oss << tbExpChipGels
				.infoStringContents("tbExpChipGels", fmtFunc) << '\n';
			oss << tbGotChipGels
				.infoStringContents("tbGotChipGels", fmtFunc) << '\n';
		}

		bool const lrOkay
			{ (lrGotChipGels.hwSize() == lrExpChipGels.hwSize())
			&& std::equal
				( lrGotChipGels.cbegin(), lrGotChipGels.cend()
				, lrExpChipGels.cbegin()
				, nearlyEqualGradels
				)
			};

		if (! lrOkay)
		{
			oss << "Failure of lr*ChipGels test\n";
			oss << lrPixels
				.infoStringContents("lrPixels", "%5.0f") << '\n';
			oss << lrGradels
				.infoStringContents("lrGradels", fmtFunc) << '\n';
			oss << lrExpChipGels
				.infoStringContents("lrExpChipGels", fmtFunc) << '\n';
			oss << lrGotChipGels
				.infoStringContents("lrGotChipGels", fmtFunc) << '\n';
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

	test0(oss);
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

