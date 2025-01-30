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
\brief Unit tests (and example) code for quadloco::ops::EdgeCenter
*/


#include "QuadLoco/opsCenterRefinerEdge.hpp"

#include "QuadLoco/appcenter.hpp"
#include "QuadLoco/imgQuadTarget.hpp"
#include "QuadLoco/io.hpp"
#include "QuadLoco/objCamera.hpp"
#include "QuadLoco/rasGrid.hpp"
#include "QuadLoco/simRender.hpp"

#include <Engabra>
#include <Rigibra>

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

		constexpr std::size_t formatAndPd{ 32u };
		constexpr std::size_t numOverSamp{ 64u };

		// [DoxyExample01]

		// simulate quad grid and image geometry data
		quadloco::img::QuadTarget imgQuad{};  // set by simulation
		sim::QuadData const simQuadData
			{ sim::Render::simpleQuadData
				(formatAndPd, numOverSamp)
			};
		ras::Grid<float> const & srcGrid = simQuadData.theGrid;
		img::Spot const expCenterSpot{ simQuadData.theImgQuad.centerSpot() };

		// find nominal center with symmetry filter response peak
		std::vector<std::size_t> const ringHalfSizes{ 5u, 3u };
		std::vector<ras::PeakRCV> const peakRCVs
			{ app::center::multiSymRingPeaks(srcGrid, ringHalfSizes) };

		// use edge (magnitudes) to refine center locations
		ops::CenterRefinerEdge const refiner(srcGrid);
		std::size_t const halfRadius{ 6u }; // filter searh size
		std::vector<img::Hit> edgeHits
			{ refiner.centerHits(peakRCVs, halfRadius) };
		// (re)sort hits to put strongest detection at front
		std::sort(edgeHits.rbegin(), edgeHits.rend());

		// [DoxyExample01]

//		double const peakDistinction{ ops::AllPeaks2D::distinction(peakRCVs) };
//		double const hitDistinction{ ops::AllPeaks2D::distinction(edgeHits) };

		if (edgeHits.empty())
		{
			oss << "Failure of !edgeHits.empty() test\n";
		}
		else
		{
			img::Hit const & bestHit = edgeHits.front();
			img::Spot const & gotCenterSpot = bestHit.location();

			double const tol{ 1./16. }; // algorithm/simulation limit
			if (! nearlyEquals(gotCenterSpot, expCenterSpot, tol))
			{
				oss << "Failure of gotCenterSpot test\n";
				oss << "exp: " << expCenterSpot << '\n';
				oss << "got: " << gotCenterSpot << '\n';
			}
		}

(void)io::writeStretchPGM("srcGrid.pgm", srcGrid);

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

