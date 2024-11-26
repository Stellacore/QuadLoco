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
\brief Unit tests (and example) code for quadloco::ops::grid
*/


#include "opsgrid.hpp"

#include "rasGrid.hpp"
#include "simgrid.hpp"

#include <algorithm>
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

		// simulate grid with strong edge constistent with expEdgel
		ras::SizeHW const hwSize{ 1024u, 2048u };
		img::Edgel const expEdgel
			{ img::Spot{ 500u, 1000u }  // edge through this location
			, img::Grad{ -1., 3. } // direction of gradient the location
			};
		ras::Grid<float> const pixGrid
			{ sim::gridWithEdge(hwSize, expEdgel) };
		ras::Grid<img::Grad> const gradGrid
			{ ops::grid::gradientGridFor(pixGrid) };

		// extract edgels from grid data

		// get edgel instances for all nonzero gradients
		std::vector<img::Edgel> const allEdgels
			{ ops::grid::allEdgelsFrom(gradGrid) };
		// get only edgel instances that are strongly linked to adjacent ones
		std::vector<img::Edgel> const linkEdgels
			{ ops::grid::linkedEdgelsFrom(gradGrid) };

		// [DoxyExample01]

		// TODO replace this with real test code
		std::string const fname(__FILE__);
		bool const isTemplate{ (std::string::npos != fname.find("/_.cpp")) };
		if (! isTemplate)
		{
			oss << "Failure to implement real test\n";
		}
	}

	//! cehck computation of radient grids
	void
	test2
		( std::ostream & oss
		)
	{
		// [DoxyExample02]

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

		// Step sizes used to evaluate gradient for this test
		constexpr std::size_t stepHalf{ 1u };
		constexpr std::size_t stepFull{ 2u * stepHalf };

		// set foreground for bottom half of the horizontal edge grid
		std::fill(tbPixels.beginRow(ndxHalf), tbPixels.end(), foreVal);
		quadloco::img::Grad const tbExpGrad{ 10./(double)stepFull, 0. };

		// Use ChipSpec to set right half foreground for vertical edge grid
		quadloco::img::ChipSpec const lrFillSpec
			{ quadloco::ras::RowCol{ 0u, lrPixels.wide()/2u }
			, quadloco::ras::SizeHW{ lrPixels.high(), lrPixels.wide()/2u }
			};
		quadloco::ras::grid::setSubGridValues(&lrPixels, lrFillSpec, foreVal);
		quadloco::img::Grad const lrExpGrad{ 0., 10./(double)stepFull };

		// Compute edge gradient across stepFull pixels

		// gradient magnitude prop to:
		// [gridVal(rc + stepHalf) - gridVal(rc - stepHalf)] / (2*stepHalf)
		// Vertical grid gradients
		quadloco::ras::Grid<quadloco::img::Grad> const lrGrads
			{ quadloco::ops::grid::gradientGridFor(lrPixels, stepHalf) };
		// Horizontal grid gradients
		quadloco::ras::Grid<quadloco::img::Grad> const tbGrads
			{ quadloco::ops::grid::gradientGridFor(tbPixels, stepHalf) };

		// [DoxyExample02]
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

