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
\brief Unit tests (and example) code for quadloco::houghHoughAB
*/


#include "cast.hpp"
#include "datCircle.hpp"
#include "datGrid.hpp"
#include "datRowCol.hpp"
#include "datSpot.hpp"
#include "houghAdderAD.hpp"
#include "houghParmAD.hpp"
#include "pixEdgel.hpp"
#include "pixGrad.hpp"
#include "pixgrid.hpp"
#include "pix.hpp"
#include "simgrid.hpp"

#include "datSpan.hpp"

#include <Engabra>

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
		using namespace quadloco;

		// [DoxyExample01]

		using namespace quadloco;

		// a well-defined edge for use in generating a simulated raster step
		pix::Edgel const expEdgel
			{ pix::Spot{ 3., 4. }
			, pix::Grad{ 2., 4. }
			};

		// simulate image with a very strong edge (constistent with expEdgel)
		dat::SizeHW const hwSize{ 8u, 10u };
		dat::Grid<float> const pixGrid
			{ sim::gridWithEdge(hwSize, expEdgel) };

		// compute a full gradient grid - each cell has gradient of pixGrid
		dat::Grid<pix::Grad> const gradGrid
			{ pix::grid::gradientGridFor(pixGrid) };

		// expected configuration
		dat::Circle const boundingCircle
			{ dat::Circle::circumScribing(gradGrid.hwSize()) };
		hough::ParmAD const expMaxAD
			{ hough::ParmAD::from(expEdgel, boundingCircle) };

		// Determine accumulation buffer size (crudely)
		std::size_t const sizeAD{ hwSize.perimeter() / 4u };
		dat::SizeHW const adSize{ sizeAD, sizeAD };

		// Setup accumulator
		hough::AdderAD adder(adSize);

		// accumulate Grad values into Hough A(lpha)-D(elta) buffer
		for (dat::Grid<pix::Grad>::const_iterator
			iter{gradGrid.cbegin()} ; gradGrid.cend() != iter ; ++iter)
		{
			// note original grid row/col location and gradient
			pix::Spot const spot
				{ cast::pixSpot(gradGrid.datRowColFor(iter)) };
			pix::Grad const & grad = *iter;

			// construct Edgel and use to determine ParmAD values
			pix::Edgel const edgel{ spot, grad };
			hough::ParmAD const parmAD
				{ hough::ParmAD::from(edgel, boundingCircle) };

			// add edge magnitude into adGrid cell(s)
			float const gradMag{ magnitude(edgel.gradient()) };
			adder.add(parmAD, gradMag);
		}

		// find max value (for this test data, there's only one max
		dat::Grid<float> const & gridAD = adder.grid();
		dat::Grid<float>::const_iterator const itMax
			{ std::max_element(gridAD.cbegin(), gridAD.cend()) };
		dat::RowCol gotRowColMax{};
		if (gridAD.cend() != itMax)
		{
			gotRowColMax = gridAD.datRowColFor(itMax);
		}
		// this should be close to expected parameter values
		dat::RowCol const expRowColMax{ adder.datRowColForAD(expMaxAD) };

		// [DoxyExample01]


		pix::Edgel const gotEdgel{ gotRowColMax, expEdgel.gradient() };
		hough::ParmAD const gotMaxAD
			{ hough::ParmAD::from(gotEdgel, boundingCircle) };
//		hough::ParmAD const expMaxAD
//			{ hough::ParmAD::from(expEdgel, boundingCircle) };
		if (! nearlyEquals(gotEdgel, expEdgel))
		{
			oss << "Failure of Edgel test(1)\n";
			oss << "exp: " << expEdgel << '\n';
			oss << "got: " << gotEdgel << '\n';
		}

		if (! isValid(boundingCircle))
		{
			oss << "Failure of valid boundingCircle test\n";
			oss << "boundingCircle: " << boundingCircle << '\n';
		}

		if (! nearlyEquals(gotMaxAD, expMaxAD))
		{
			oss << "Failure of parmMaxAD test(1)\n";
			oss << "exp: " << expMaxAD << '\n';
			oss << "got: " << gotMaxAD << '\n';
		}

std::cout << adder.grid().infoStringContents("adder", "%2.1f") << '\n';

		if (! (gotRowColMax == expRowColMax))
		{
			oss << "Failure of gotRowColMax test\n";
			oss << "exp: " << expRowColMax << '\n';
			oss << "got: " << gotRowColMax << '\n';
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

