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
#include "imgCircle.hpp"
#include "imgEdgel.hpp"
#include "imgGrad.hpp"
#include "imgSpot.hpp"
#include "opsAdderAD.hpp"
#include "opsgrid.hpp"
#include "rasGrid.hpp"
#include "rasRowCol.hpp"
#include "sigParmAD.hpp"
#include "simgrid.hpp"

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
		ras::SizeHW const hwSize{ 17u, 19u };

		// a well-defined edge for use in generating a simulated raster step
		img::Edgel const expEdgel
			{ img::Spot
				{ (double)(hwSize.high()/2u), (double)(hwSize.wide()/2u) }
			, img::Grad{ 2., 3. }
			};

		// simulate image with a very strong edge (constistent with expEdgel)
		ras::Grid<float> const pixGrid
			{ sim::gridWithEdge(hwSize, expEdgel) };

		// compute a full gradient grid - each cell has gradient of pixGrid
		ras::Grid<img::Grad> const gradGrid
			{ ops::grid::gradientGridFor(pixGrid) };

		// expected configuration
		img::Circle const boundingCircle
			{ img::Circle::circumScribing(gradGrid.hwSize()) };
		sig::ParmAD const expMaxAD
			{ sig::ParmAD::from(expEdgel, boundingCircle) };

		// Determine accumulation buffer size (crudely)
		std::size_t const sizeAD{ hwSize.perimeter() / 4u };
		ras::SizeHW const adSize{ sizeAD, sizeAD };

		// Setup accumulator
		ops::AdderAD adder(adSize);

		// accumulate Grad values into Hough A(lpha)-D(elta) buffer
		for (ras::Grid<img::Grad>::const_iterator
			iter{gradGrid.cbegin()} ; gradGrid.cend() != iter ; ++iter)
		{
			// note original grid row/col location and gradient
			img::Spot const spot
				{ cast::imgSpot(gradGrid.rasRowColFor(iter)) };
			img::Grad const & grad = *iter;

			// construct Edgel and use to determine ParmAD values
			img::Edgel const edgel{ spot, grad };
			sig::ParmAD const parmAD
				{ sig::ParmAD::from(edgel, boundingCircle) };

			// add edge magnitude into adGrid cell(s)
			double const gradMag{ magnitude(edgel.gradient()) };
			adder.add(parmAD, gradMag);
		}

		// find max value (for this test data, there's only one max
		ras::Grid<float> const & gridAD = adder.grid();
		ras::Grid<float>::const_iterator const itMax
			{ std::max_element(gridAD.cbegin(), gridAD.cend()) };
		ras::RowCol gotRowColMax{};
		if (gridAD.cend() != itMax)
		{
			gotRowColMax = gridAD.rasRowColFor(itMax);
		}

		// Hough parameter (alpha,delta) values for accum cell w/ max value
		img::Spot const gotSpotMax{ cast::imgSpot(gotRowColMax) };
		sig::ParmAD const gotParmAD{ adder.sigParmADFor(gotSpotMax) };

		// Hough (alpha,delta) values expected for simulation generating edgel
		sig::ParmAD const expParmAD
			{ sig::ParmAD::from(expEdgel, boundingCircle) };

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

		double const tolAD{ 2. / (.5 * hwSize.diagonal()) };
		if (! nearlyEquals(gotParmAD, expParmAD, tolAD))
		{
			double const gotAlpha{ gotParmAD.alpha() };
			double const expAlpha{ expParmAD.alpha() };
			double const difAlpha{ gotAlpha - expAlpha };

			double const gotDelta{ gotParmAD.delta() };
			double const expDelta{ expParmAD.delta() };
			double const difDelta{ gotDelta - expDelta };

			using engabra::g3::io::fixed;
			oss << "Failure of gotParmAD test\n";
			oss << "expAlpha: " << fixed(expAlpha) << '\n';
			oss << "gotAlpha: " << fixed(gotAlpha) << '\n';
			oss << "expDelta: " << fixed(expDelta) << '\n';
			oss << "gotDelta: " << fixed(gotDelta) << '\n';
			oss << "difAlpha: " << fixed(difAlpha) << '\n';
			oss << "difDelta: " << fixed(difDelta) << '\n';
			oss << "   tolAD: " << fixed(tolAD) << '\n';
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

