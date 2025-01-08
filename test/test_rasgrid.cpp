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
\brief Unit tests (and example) code for quadloco::ras::grid:: functions
*/


#include "rasgrid.hpp"
#include "opsgrid.hpp" // /TODO remove

#include "imgGrad.hpp"
#include "pix.hpp"
#include "rasChipSpec.hpp"
#include "rasgrid.hpp"
#include "rasGrid.hpp"

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
		quadloco::ras::Grid<char> grid(10u, 7u);
		std::fill(grid.begin(), grid.end(), '.');

		// fill outer two rows and outer two columns
		// This uses a slightly faster algorithm than the individual
		// fill operations below
		quadloco::ras::grid::fillBorder
			(grid.begin(), grid.hwSize(), 2u, 'x');

		// fill outer one cell thick perimeter - one edge at a time
		quadloco::ras::grid::fillInitRows
			(grid.begin(), grid.hwSize(), 1u, 'W');
		quadloco::ras::grid::fillInitCols
			(grid.begin(), grid.hwSize(), 1u, 'W');
		quadloco::ras::grid::fillLastCols
			(grid.begin(), grid.hwSize(), 1u, 'W');
		quadloco::ras::grid::fillLastRows
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
		quadloco::ras::SizeHW const hwSize{ ndxFull, ndxFull };
		// one with vertical, one with horizontal edges
		quadloco::ras::Grid<float> tbPixels(hwSize); // Top-to-Bot edge
		quadloco::ras::Grid<float> lrPixels(hwSize); // Lft-to-Rgt edge
		constexpr float backVal{ -15.25f };
		constexpr float foreVal{  -5.25f };
		// initialize both grids to background values
		std::fill(tbPixels.begin(), tbPixels.end(), backVal);
		std::fill(lrPixels.begin(), lrPixels.end(), backVal);

		// step edges generate 2 cell wide response
		constexpr double edgeSize{ 2. };

		// set foreground for bottom half of the horizontal edge grid
		std::fill(tbPixels.beginRow(ndxHalf), tbPixels.end(), foreVal);
		quadloco::img::Grad const tbExpGrad{ 10./edgeSize, 0. };

		// Use ChipSpec to set right half foreground for vertical edge grid
		quadloco::ras::ChipSpec const lrFillSpec
			{ quadloco::ras::RowCol{ 0u, lrPixels.wide()/2u }
			, quadloco::ras::SizeHW{ lrPixels.high(), lrPixels.wide()/2u }
			};
		quadloco::ras::grid::setSubGridValues(&lrPixels, lrFillSpec, foreVal);
		quadloco::img::Grad const lrExpGrad{ 0., 10./edgeSize };

		// Compute edge gradient for each source cell

		// gradient magnitude prop to:
		// Vertical grid gradients
		quadloco::ras::Grid<quadloco::img::Grad> const lrGrads
			{ quadloco::ops::grid::gradientGridBy8x(lrPixels) };
		// Horizontal grid gradients
		quadloco::ras::Grid<quadloco::img::Grad> const tbGrads
			{ quadloco::ops::grid::gradientGridBy8x(tbPixels) };

		// [DoxyExample01]


		using namespace quadloco::img;
		using namespace quadloco::ras;

		// Extract computed gradient values within edge regions
		ChipSpec const tbChipSpec{ RowCol{ 3u, 1u }, SizeHW{ 2u, 6u } };
		Grid<Grad> const tbGotChipGrads
			{ grid::subGridValuesFrom(tbGrads, tbChipSpec) };
		//
		ChipSpec const lrChipSpec{ RowCol{ 1u, 3u }, SizeHW{ 6u, 2u } };
		Grid<Grad> const lrGotChipGrads
			{ grid::subGridValuesFrom(lrGrads, lrChipSpec) };

		// Create expected chips populated with expected grad values
		Grid<Grad> tbExpChipGrads(tbGotChipGrads.hwSize());
		std::fill(tbExpChipGrads.begin(), tbExpChipGrads.end(), tbExpGrad);
		//
		Grid<Grad> lrExpChipGrads(lrGotChipGrads.hwSize());
		std::fill(lrExpChipGrads.begin(), lrExpChipGrads.end(), lrExpGrad);

		// Check if extracted edge values match expected ones

		Vector<double>::Formatter const fmtFunc{ "{:4.1}" };

		std::function<bool(Grad const & gdelA, Grad const & gdelB)>
			const nearlyEqualGrads
			{ [] (Grad const & gdelA, Grad const & gdelB)
				{ return nearlyEquals(gdelA, gdelB); }
			};

		bool const tbOkay
			{ (tbGotChipGrads.hwSize() == tbExpChipGrads.hwSize())
			&& std::equal
				( tbGotChipGrads.cbegin(), tbGotChipGrads.cend()
				, tbExpChipGrads.cbegin()
				, nearlyEqualGrads
				)
			};

		if (! tbOkay)
		{
			oss << "Failure of tb*ChipGrads test\n";
			oss << tbPixels
				.infoStringContents("tbPixels", "%5.0f") << '\n';
			oss << tbGrads
				.infoStringContents("tbGrads", fmtFunc) << '\n';
			oss << tbExpChipGrads
				.infoStringContents("tbExpChipGrads", fmtFunc) << '\n';
			oss << tbGotChipGrads
				.infoStringContents("tbGotChipGrads", fmtFunc) << '\n';
		}

		bool const lrOkay
			{ (lrGotChipGrads.hwSize() == lrExpChipGrads.hwSize())
			&& std::equal
				( lrGotChipGrads.cbegin(), lrGotChipGrads.cend()
				, lrExpChipGrads.cbegin()
				, nearlyEqualGrads
				)
			};

		if (! lrOkay)
		{
			oss << "Failure of lr*ChipGrads test\n";
			oss << lrPixels
				.infoStringContents("lrPixels", "%5.0f") << '\n';
			oss << lrGrads
				.infoStringContents("lrGraGrads", fmtFunc) << '\n';
			oss << lrExpChipGrads
				.infoStringContents("lrExpChipGrads", fmtFunc) << '\n';
			oss << lrGotChipGrads
				.infoStringContents("lrGotChipGrads", fmtFunc) << '\n';
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
		quadloco::ras::SizeHW const hwSize{ 4u, 5u };
		quadloco::ras::Grid<double> values{ hwSize };
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
		quadloco::img::Spot const at
			{ .25 + (double)rowNdx1
			, .75 + (double)rowNdx2
			};
		double const gotVal{ quadloco::ras::grid::bilinValueAt(values, at) };

		// [DoxyExample02]

		quadloco::img::Spot const out
			{ 2.25 + (double)rowNdx1
			,  .75 + (double)rowNdx2
			};
		double const outVal{ quadloco::ras::grid::bilinValueAt(values, out) };
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
		ras::Grid<std::complex<double> > cplxGrid(4u, 5u);
		std::complex<double> const expValue{ 100., -200. };
		std::fill	
			( cplxGrid.begin(), cplxGrid.end()
			, expValue
			);
		std::complex<double> const gotValue
			{ ras::grid::bilinValueAt(cplxGrid, img::Spot{ 2., 3. }) };
		// interp should be exact with the integer values used here
		if (! (gotValue == expValue))
		{
			oss << "Failure of bilinear complex interp test(3)\n";
			oss << "exp: " << expValue << '\n';
			oss << "got: " << gotValue << '\n';
		}
	}

	//! check min,max finding
	void
	test3
		( std::ostream & oss
		)
	{
		using namespace quadloco;

		ras::Grid<double> const aNull{};
		using ItInt = ras::Grid<double>::const_iterator;
		std::pair<ItInt, ItInt> const itMM
			{ ras::grid::minmax_valid(aNull.cbegin(), aNull.cend()) };
		if (! ((aNull.cend() == itMM.first) && (aNull.cend() == itMM.second)))
		{
			oss << "Failure of aNull minmax_valid test\n";
		}

		// [DoxyExample06]

		constexpr float fNan{ std::numeric_limits<float>::quiet_NaN() };

		ras::Grid<float> fGrid(3u, 3u);
		std::fill(fGrid.begin(), fGrid.end(), fNan);
		float const expMin{ -3.f };
		float const expMax{  5.f };
		fGrid(1u, 2u) = expMin;
		fGrid(1u, 1u) = .5f*(expMin + expMax);
		fGrid(2u, 2u) = expMax;

		using ItFlt = ras::Grid<float>::const_iterator;
		std::pair<ItFlt, ItFlt> const itPair
			{ ras::grid::minmax_valid(fGrid.cbegin(), fGrid.cend()) };
		float gotMin{ fNan };
		float gotMax{ fNan };
		if ((fGrid.cend() != itPair.first) && (fGrid.cend() != itPair.second))
		{
			gotMin = *(itPair.first);
			gotMax = *(itPair.second);
		}

		// [DoxyExample06]

		// compare should be exact (no computations, just copies)
		if (! ((gotMin == expMin) && (gotMax == expMax)))
		{
			oss << "Failure of fGrid minmax_valid test\n";
			oss << "expMin: " << expMin << '\n';
			oss << "expMax: " << expMax << '\n';
			oss << "gotMin: " << gotMin << '\n';
			oss << "gotMax: " << gotMax << '\n';
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
	test3(oss);

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

