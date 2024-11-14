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
#include "pixGrad.hpp"
#include "pixgrid.hpp"

#include <algorithm>
#include <complex>
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
		quadloco::pix::Grad const tbExpGrad{ 10./(double)stepFull, 0. };

		// Use ChipSpec to set right half foreground for vertical edge grid
		quadloco::dat::ChipSpec const lrFillSpec
			{ quadloco::dat::RowCol{ 0u, lrPixels.wide()/2u }
			, quadloco::dat::SizeHW{ lrPixels.high(), lrPixels.wide()/2u }
			};
		quadloco::pix::grid::setSubGridValues(&lrPixels, lrFillSpec, foreVal);
		quadloco::pix::Grad const lrExpGrad{ 0., 10./(double)stepFull };

		// Compute edge gradient across stepFull pixels

		// gradient magnitude prop to:
		// [gridVal(rc + stepHalf) - gridVal(rc - stepHalf)] / (2*stepHalf)
		// Vertical grid gradients
		quadloco::dat::Grid<quadloco::pix::Grad> const lrGrads
			{ quadloco::pix::grid::gradientGridFor(lrPixels, stepHalf) };
		// Horizontal grid gradients
		quadloco::dat::Grid<quadloco::pix::Grad> const tbGrads
			{ quadloco::pix::grid::gradientGridFor(tbPixels, stepHalf) };

		// [DoxyExample01]


		using namespace quadloco::dat;
		using namespace quadloco::pix;

		// Extract computed gradient values within edge regions
		ChipSpec const tbChipSpec{ RowCol{ 3u, 1u }, SizeHW{ 2u, 6u } };
		Grid<Grad> const tbGotChipGels
			{ grid::subGridValuesFrom(tbGrads, tbChipSpec) };
		//
		ChipSpec const lrChipSpec{ RowCol{ 1u, 3u }, SizeHW{ 6u, 2u } };
		Grid<Grad> const lrGotChipGels
			{ grid::subGridValuesFrom(lrGrads, lrChipSpec) };

		// Create expected chips populated with expected grad values
		Grid<Grad> tbExpChipGels(tbGotChipGels.hwSize());
		std::fill(tbExpChipGels.begin(), tbExpChipGels.end(), tbExpGrad);
		//
		Grid<Grad> lrExpChipGels(lrGotChipGels.hwSize());
		std::fill(lrExpChipGels.begin(), lrExpChipGels.end(), lrExpGrad);

		// Check if extracted edge values match expected ones

		Vec2D<float>::Formatter const fmtFunc{ "{:4.1f}" };

		std::function<bool(Grad const & gdelA, Grad const & gdelB)>
			const nearlyEqualGrads
			{ [] (Grad const & gdelA, Grad const & gdelB)
				{ return nearlyEquals(gdelA, gdelB); }
			};

		bool const tbOkay
			{ (tbGotChipGels.hwSize() == tbExpChipGels.hwSize())
			&& std::equal
				( tbGotChipGels.cbegin(), tbGotChipGels.cend()
				, tbExpChipGels.cbegin()
				, nearlyEqualGrads
				)
			};

		if (! tbOkay)
		{
			oss << "Failure of tb*ChipGels test\n";
			oss << tbPixels
				.infoStringContents("tbPixels", "%5.0f") << '\n';
			oss << tbGrads
				.infoStringContents("tbGrads", fmtFunc) << '\n';
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
				, nearlyEqualGrads
				)
			};

		if (! lrOkay)
		{
			oss << "Failure of lr*ChipGels test\n";
			oss << lrPixels
				.infoStringContents("lrPixels", "%5.0f") << '\n';
			oss << lrGrads
				.infoStringContents("lrGraGrads", fmtFunc) << '\n';
			oss << lrExpChipGels
				.infoStringContents("lrExpChipGels", fmtFunc) << '\n';
			oss << lrGotChipGels
				.infoStringContents("lrGotChipGels", fmtFunc) << '\n';
		}
	}

	//! Check bilinear interpolation
	void
	test2
		( std::ostream & oss
		)
	{
		// [DoxyExample02]

		// grid of samples (minimum example here)
		quadloco::dat::SizeHW const hwSize{ 4u, 5u };
		quadloco::dat::Grid<double> values{ hwSize };
		std::fill(values.begin(), values.end(), quadloco::pix::null<double>());
		constexpr std::size_t rowNdx1{ 1u };
		constexpr std::size_t rowNdx2{ rowNdx1 + 1u };
		constexpr std::size_t colNdx1{ 2u };
		constexpr std::size_t colNdx2{ colNdx1 + 1u };
		values(rowNdx1, colNdx1) = 17.;
		values(rowNdx2, colNdx1) = 11.;
		values(rowNdx1, colNdx2) = 23.;
		values(rowNdx2, colNdx2) = 29.;

		// sample at arbitrary location (inside the grid, else returns null)
		quadloco::dat::Spot const at
			{ .25 + (double)rowNdx1
			, .75 + (double)rowNdx2
			};
		double const gotVal{ quadloco::pix::grid::bilinValueAt(values, at) };

		// [DoxyExample02]

		quadloco::dat::Spot const out
			{ 2.25 + (double)rowNdx1
			,  .75 + (double)rowNdx2
			};
		double const outVal{ quadloco::pix::grid::bilinValueAt(values, out) };
		if (engabra::g3::isValid(outVal))
		{
			oss << "Failure of out of bounds not valid test(3)\n";
			oss << "exp: " << quadloco::pix::null<double>() << '\n';
			oss << "got: " << outVal << '\n';
		}

		//! wikipedia: https://en.wikipedia.org/wiki/Bilinear_interpolation
		double const & fQ11 = values(rowNdx1, colNdx1);
		double const & fQ21 = values(rowNdx2, colNdx1);
		double const & fQ12 = values(rowNdx1, colNdx2);
		double const & fQ22 = values(rowNdx2, colNdx2);
		//
		double const x0{ at[0] };
		double const y0{ at[1] };
		//
		double const x1{ (double)rowNdx1 };
		double const y1{ (double)colNdx1 };
		double const x2{ (double)rowNdx2 };
		double const y2{ (double)colNdx2 };
		//
		double const x20{ x2 - x0 };
		double const x01{ x0 - x1 };
		double const y20{ y2 - y0 };
		double const y01{ y0 - y1 };
		//
		double const x21{ x2 - x1 };
		double const y21{ y2 - y1 };
		double const scale{ 1. / (x21 * y21) };
		double const sum
			{ fQ11*x20*y20
			+ fQ21*x01*y20
			+ fQ12*x20*y01
			+ fQ22*x01*y01
			};
		double const expVal{ scale * sum };
		if (! engabra::g3::nearlyEquals(gotVal, expVal))
		{
			oss << "Failure of bilinear double interp test(3)\n";
			oss << "exp: " << expVal << '\n';
			oss << "got: " << gotVal << '\n';
			oss << values.infoStringContents("values", "%6.2f") << '\n';

		}

		// check instantation with a complex type
		using namespace quadloco;
		using namespace quadloco::dat;
		using namespace quadloco::pix::grid;
		dat::Grid<std::complex<double> > cplxGrid(4u, 5u);
		std::complex<double> const expValue{ 100., -200. };
		std::fill	
			( cplxGrid.begin(), cplxGrid.end()
			, expValue
			);
		std::complex<double> const gotValue
			{ pix::grid::bilinValueAt(cplxGrid, Spot{ 2., 3. }) };
		// interp should be exact with the integer values used here
		if (! (gotValue == expValue))
		{
			oss << "Failure of bilinear complex interp test(3)\n";
			oss << "exp: " << expValue << '\n';
			oss << "got: " << gotValue << '\n';
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
	test2(oss);

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

