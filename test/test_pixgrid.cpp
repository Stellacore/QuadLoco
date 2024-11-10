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

		// using tiny data grids for easy testing
		constexpr std::size_t ndxHalf{ 4u };
		constexpr std::size_t ndxFull{ 2u*ndxHalf };
		quadloco::dat::SizeHW const hwSize{ ndxFull, ndxFull };
		// one with vertical, one with horizontal edges
		quadloco::dat::Grid<float> vPixels(hwSize); // Vertical edge
		quadloco::dat::Grid<float> hPixels(hwSize); // Horizontal edge
		constexpr float backVal{ -15.25f };
		constexpr float foreVal{  -5.25f };
		// initialize both grids to background values
		std::fill(vPixels.begin(), vPixels.end(), backVal);
		std::fill(hPixels.begin(), hPixels.end(), backVal);

		// set foreground for bottom half of the horizontal edge grid
		std::fill(hPixels.beginRow(ndxHalf), hPixels.end(), foreVal);

		// set foreground for right half of the vertical edge grid
		for (std::size_t row{0u} ; row < hwSize.high() ; ++row)
		{
			// and right half of vertical edge grid to foreground
			std::fill
				( vPixels.beginRow(row) + ndxHalf
				, vPixels.endRow(row)
				, foreVal
				);
		}

		// compute edge gradient across stepFull pixels
		// Gradient magnitude prop to:
		// [gridVal(rc + stepHalf) - gridVal(rc - stepHalf)] / (2*stepHalf)
		constexpr std::size_t stepHalf{ 1u };
		constexpr std::size_t stepFull{ 2u * stepHalf };
		// Vertical grid gradients
		quadloco::dat::Grid<quadloco::pix::Gradel> const vGradels
			{ quadloco::pix::grid::gradelGridFor(vPixels, stepHalf) };
		// Horizontal grid gradients
		quadloco::dat::Grid<quadloco::pix::Gradel> const hGradels
			{ quadloco::pix::grid::gradelGridFor(hPixels, stepHalf) };


		// [DoxyExample01]

		// last pixel of first half
		constexpr std::size_t ndxLast{ ndxHalf - 1u };
		// firat pixel of second half
		constexpr std::size_t ndxNext{ ndxHalf };

		// for edge detection across stepFull pixels (stepHalf on each side)
		constexpr std::size_t ndxEdgeBeg{ ndxLast - (stepHalf-1u) };
		constexpr std::size_t ndxEdgeEnd{ ndxNext + (stepHalf-1u) };

std::cout << "ndxEdgeBeg: " << ndxEdgeBeg << '\n';
std::cout << "ndxEdgeEnd: " << ndxEdgeEnd << '\n';

		for (std::size_t row{0u} ; row < hwSize.high() ; ++row)
		{
			for (std::size_t col{0u} ; col < hwSize.wide() ; ++col)
			{
			}
		}

		using quadloco::dat::Grid;
		using quadloco::pix::Gradel;
		std::cout << vPixels.infoStringContents("vPixels", "%5.0f") << '\n';
		std::cout << hPixels.infoStringContents("hPixels", "%5.0f") << '\n';
		auto const fmtFunc
			{ [] (Gradel const & elem)
				{ return std::format("({:5.3f},{:5.3f})", elem[0], elem[1]); }
			};
		std::cout << vGradels.infoStringContents("vGradels", fmtFunc) << '\n';
		std::cout << hGradels.infoStringContents("hGradels", fmtFunc) << '\n';

oss << "implement me\n";

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
//	test1(oss);

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

