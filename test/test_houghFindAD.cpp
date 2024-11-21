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

		// (simulated) image size
		dat::SizeHW const hwSize{ 17u, 19u };

		// a well-defined edge for use in generating a simulated raster step
		pix::Edgel const expEdgel
			{ pix::Spot{ (float)(hwSize.high()/2u), (float)(hwSize.wide()/2u) }
			, pix::Grad{ 2., 3. }
			};

		// simulate image with a very strong edge (constistent with expEdgel)
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

		// Hough parameter (alpha,delta) values for accum cell w/ max value
		dat::Spot const gotSpotMax{ cast::datSpot(gotRowColMax) };
		hough::ParmAD const gotParmAD{ adder.houghParmADFor(gotSpotMax) };

		// Hough (alpha,delta) values expected for simulation generating edgel
		hough::ParmAD const expParmAD
			{ hough::ParmAD::from(expEdgel, boundingCircle) };

		// [DoxyExample01]

		/*
		std::cout << pixGrid.infoStringContents("pixGrid", "%2.1f") << '\n';
		std::cout << "gotRowColMax: " << gotRowColMax << '\n';
		std::cout << "gotSpotMax: " << gotSpotMax << '\n';
		std::cout << "expParmAD: " << expParmAD << '\n';
		std::cout << "gotParmAD: " << gotParmAD << '\n';
		std::cout << adder.grid().infoStringContents("adder", "%2.1f") << '\n';
		*/

		if (! isValid(boundingCircle))
		{
			oss << "Failure of valid boundingCircle test\n";
			oss << "boundingCircle: " << boundingCircle << '\n';
		}

		// the alpha parameters should be similar but can wrap around
		// therefore compare components of a (unit) vector at angle alpha
		dat::Spot const gotAlphaComps 
			{ std::cos(gotParmAD.alpha())
			, std::sin(gotParmAD.alpha())
			};
		dat::Spot const expAlphaComps 
			{ std::cos(expParmAD.alpha())
			, std::sin(expParmAD.alpha())
			};
		double const tolAlpha{ 1. / (.5 * hwSize.diagonal()) };
		if (! nearlyEquals(gotAlphaComps, expAlphaComps, tolAlpha))
		{
			dat::Spot const difAlphaComps{ gotAlphaComps - expAlphaComps };
			using engabra::g3::io::fixed;
			oss << "Failure of gotAlphaComps test\n";
			oss << "gotAlpha: " << fixed(gotParmAD.alpha()) << '\n';
			oss << "expAlpha: " << fixed(expParmAD.alpha()) << '\n';
			oss << "gotAlphaComps: " << gotAlphaComps << '\n';
			oss << "expAlphaComps: " << expAlphaComps << '\n';
			oss << "difAlphaComps: " << difAlphaComps << '\n';
			oss << "      ... mag: " << magnitude(difAlphaComps) << '\n';
			oss << "      ... tol: " << fixed(tolAlpha) << '\n';
		}

		// the delta parameters shouldn't wrap around (for sane data)
		// ??? not sure what this really should be ???
		//     so just double alpha tolerance
		double const gotDelta{ gotParmAD.delta() };
		double const expDelta{ expParmAD.delta() };
		double const tolDelta{ 2. / (.5 * hwSize.diagonal()) };
		if (! engabra::g3::nearlyEquals(gotDelta, expDelta, tolDelta))
		{
			double const difDelta{ gotDelta - expDelta };
			using engabra::g3::io::fixed;
			oss << "Failure of gotDelta test\n";
			oss << "exp: " << fixed(expDelta) << '\n';
			oss << "got: " << fixed(gotDelta) << '\n';
			oss << "dif: " << fixed(difDelta) << '\n';
			oss << "tol: " << fixed(tolDelta) << '\n';
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

