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


#include "appCenters.hpp"
#include "imgQuadTarget.hpp"
#include "imgSpot.hpp"
#include "io.hpp"
#include "objCamera.hpp"
#include "objQuadTarget.hpp"
#include "opsAllPeaks2D.hpp"
#include "opsCenterRefinerSSD.hpp"
#include "rasGrid.hpp"
#include "rasPeakRCV.hpp"
#include "rasSizeHW.hpp"
#include "simConfig.hpp"
#include "simRender.hpp"

#include <iostream>
#include <sstream>
#include <vector>


namespace
{
	//! Simulate a strong signal quad image
	inline
	quadloco::ras::Grid<float>
	simulatedQuadGrid
		( quadloco::img::QuadTarget * const & ptSigQuad = nullptr
		)
	{
		using namespace quadloco;
		using namespace quadloco::obj;
		sim::Config const config
			{ obj::QuadTarget
				( 1.00
			//	, QuadTarget::None
			//	, QuadTarget::WithSurround
			//	, QuadTarget::WithTriangle
				, QuadTarget::WithSurround | QuadTarget::WithTriangle
				)
			//
//			, obj::Camera::isoCam(128u)
			, obj::Camera::isoCam( 32u)
//			, obj::Camera::isoCam( 31u)
//			, obj::Camera::isoCam( 16u)
//			, obj::Camera::isoCam( 15u)
//			, obj::Camera::isoCam(  8u)
//			, obj::Camera::isoCam(  7u)
			//
			, rigibra::Transform
				{ engabra::g3::Vector{ 0., 0., 1. }
				, rigibra::Attitude
					{ rigibra::PhysAngle
						{ engabra::g3::BiVector{ 0., 0., 0. } }
					}
				}
			};
		using namespace quadloco::sim;
		sim::Render const render
			( config
		//	, Sampler::None
			, Sampler::AddSceneBias | Sampler::AddImageNoise
		//	, Sampler::AddImageNoise
			);
		std::size_t const numOverSample{ 64u };
		if (ptSigQuad)
		{
			*ptSigQuad = render.imgQuadTarget();
		}
		return render.quadGrid(numOverSample);
	}

	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		using namespace quadloco;

		// simulate quad target image and extract edgels
		quadloco::img::QuadTarget imgQuad{};  // set by simulation
		ras::Grid<float> const srcGrid{ simulatedQuadGrid(&imgQuad) };
		img::Spot const expCenterSpot{ imgQuad.centerSpot() };

		//(void)io::writeStretchPGM("srcGrid.pgm", srcGrid);

		// [DoxyExample01a]

		// find nominal center with symmetry filter response peak
		std::vector<ras::PeakRCV> const peakRCVs
			{ app::multiSymRingPeaks(srcGrid) };
		double const distinction{ ops::AllPeaks2D::distinction(peakRCVs) };

		// [DoxyExample01a]

std::cout << "expCenterSpot: " << expCenterSpot << '\n';
std::cout << "peakRCVs.size: " << peakRCVs.size() << '\n';
std::cout << "  distinction: " << engabra::g3::io::fixed(distinction) << '\n';

		if (peakRCVs.empty())
		{
			oss << "Failure of peakRCVs.empty() test\n";
		}
		else // if (! peakRCVs.empty())
		{
			// [DoxyExample01b]

			ras::RowCol const & nominalCenterRC = peakRCVs.front().theRowCol;
			ops::CenterRefinerSSD const refiner(&srcGrid, 2u, 6u);
			img::Spot const gotCenterSpot
				{ refiner.fitSpotNear(nominalCenterRC) };

			img::Spot const difCenterSpot{ gotCenterSpot - expCenterSpot };
			double const difMag{ magnitude(difCenterSpot) };

			// [DoxyExample01b]

using engabra::g3::io::fixed;
std::cout << " peakRCVs.front: " << peakRCVs.front() << '\n';
std::cout << "nominalCenterRC: " << nominalCenterRC << '\n';
std::cout << "  expCenterSpot: " << expCenterSpot << '\n';
std::cout << "  gotCenterSpot: " << gotCenterSpot << '\n';
std::cout << "  difCenterSpot: "
	<< difCenterSpot << "  mag: " << fixed(difMag, 2u, 2u) << '\n';
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

