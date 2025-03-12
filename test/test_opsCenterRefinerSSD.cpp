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
\brief Unit tests (and example) code for quadloco::CenterRefinerSSD
*/


#include "QuadLoco/appcenter.hpp"
#include "QuadLoco/imgQuadTarget.hpp"
#include "QuadLoco/imgSpot.hpp"
#include "QuadLoco/io.hpp"
#include "QuadLoco/objCamera.hpp"
#include "QuadLoco/objQuadTarget.hpp"
#include "QuadLoco/opsAllPeaks2D.hpp"
#include "QuadLoco/opsCenterRefinerSSD.hpp"
#include "QuadLoco/rasGrid.hpp"
#include "QuadLoco/rasPeakRCV.hpp"
#include "QuadLoco/rasSizeHW.hpp"
#include "QuadLoco/simConfig.hpp"
#include "QuadLoco/simRender.hpp"

#include <iostream>
#include <sstream>
#include <vector>


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

		// simulate quad target image and extract edgels
		quadloco::img::QuadTarget imgQuad{};  // set by simulation
		sim::QuadData const simQuadData
			{ sim::Render::simpleQuadData
				(formatAndPd, numOverSamp)
			};
		ras::Grid<float> const & srcGrid = simQuadData.theGrid;
		img::Spot const expCenterSpot{ simQuadData.theImgQuad.centerSpot() };

		// (void)io::writeStretchPGM("fooTest.pgm", srcGrid);

		//(void)io::writeStretchPGM("srcGrid.pgm", srcGrid);

		// [DoxyExample01a]

		// find nominal center with symmetry filter response peak
		std::vector<std::size_t> const ringHalfSizes{ 5u, 3u };
		std::vector<ras::PeakRCV> const peakRCVs
			{ app::center::multiSymRingPeaks(srcGrid, ringHalfSizes) };
		double const distinction{ ops::AllPeaks2D::distinction(peakRCVs) };

		// [DoxyExample01a]

		if (peakRCVs.empty())
		{
			oss << "Failure of peakRCVs.empty() test\n";
		}
		else // if (! peakRCVs.empty())
		{
			// [DoxyExample01b]

			ras::RowCol const & nominalCenterRC = peakRCVs.front().theRowCol;

			// refine the location of the symmetry peak found aboe
			ops::CenterRefinerSSD const refiner(&srcGrid, 2u, 6u);
			img::Hit const gotCenterHit
				{ refiner.fitHitNear(nominalCenterRC) };

			img::Spot const & gotCenterSpot = gotCenterHit.location();

			// [DoxyExample01b]

			// NOTE: image (radiometric) noise affects the fit center
			//       tolerance value here reflects this.
			constexpr double tol{ 5./8. }; // [pix]
			if (! nearlyEqualsAbs(gotCenterSpot, expCenterSpot, tol))
			{
				using engabra::g3::io::fixed;
				img::Spot const difCenterSpot
					{ gotCenterHit.location() - expCenterSpot };
				double const difMag{ magnitude(difCenterSpot) };
				oss << "Failure of (ssd)refined gotCenterSpot test\n";
				oss << "exp: " << expCenterSpot << '\n';
				oss << "got: " << gotCenterSpot << '\n';
				oss << "dif: " << difCenterSpot
					<< "  mag: " << fixed(difMag) << '\n';
				oss << "tol: " << fixed(tol) << '\n';
				oss << "detail:\n";
				oss << "-- peakRCVs.front: " << peakRCVs.front() << '\n';
				oss << "--nominalCenterRC: " << nominalCenterRC << '\n';
				oss << "--  peakRCVs.size: " << peakRCVs.size() << '\n';
				oss << "--    distinction: " << fixed(distinction) << '\n';
				oss << "--   gotCenterHit: " << gotCenterHit << '\n';
			}
		}

	} // test1

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

