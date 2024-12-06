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
\brief Unit tests (and example) code for quadloco::sig::EdgeEval
*/


#include "imgEdgel.hpp"
#include "io.hpp"
#include "objCamera.hpp"
#include "objQuadTarget.hpp"
#include "opsgrid.hpp"
#include "sigEdgeEval.hpp"
#include "simConfig.hpp"
#include "simRender.hpp"

#include <Engabra>
#include <Rigibra>

#include <fstream>
#include <iostream>
#include <sstream>


namespace
{
	//! Simulate a strong signal quad image
	inline
	quadloco::ras::Grid<float>
	simulatedQuadGrid
		( quadloco::sig::QuadTarget * const & ptSigQuad = nullptr
		)
	{
		using namespace quadloco;
		using namespace quadloco::obj;
		sim::Config const config
			{ obj::QuadTarget
				( 1.00
			//	, QuadTarget::None
				, QuadTarget::WithSurround | QuadTarget::WithTriangle
			//	, QuadTarget::WithTriangle
				)
			, obj::Camera
//				{ ras::SizeHW{ 128u, 128u }
//				, 300. // pd
//{ ras::SizeHW{ 16u, 16u }
//				,  55. // pd
//{ ras::SizeHW{ 10u, 10u }
//				,  40. // pd
{ ras::SizeHW{   20u,   20u }
				,   20. // pd
				}
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
			, Sampler::None
		//	, Sampler::AddSceneBias | Sampler::AddImageNoise
		//	, Sampler::AddImageNoise
			);
		std::size_t const numOverSample{ 1024u };
//std::size_t const numOverSample{    0u };
		if (ptSigQuad)
		{
			*ptSigQuad = render.sigQuadTarget();
		}
		return render.quadImage(numOverSample);
	}

	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		using namespace quadloco;

		// simulate quad target image and extract edgels
		quadloco::sig::QuadTarget sigQuad{};  // set by simulation
		ras::Grid<float> const pixGrid{ simulatedQuadGrid(&sigQuad) };
		img::Spot const expCenterSpot{ sigQuad.centerSpot() };

		// compute gradient elements
		ras::Grid<img::Grad> const gradGrid
			{ ops::grid::gradientGridFor(pixGrid) };
		// assess significant gradients in context with other
		std::vector<img::Edgel> const edgels
			{ ops::grid::linkedEdgelsFrom(gradGrid) };

		// categorize edgels as candidates for (radial) quad target edge groups
		sig::EdgeEval const edgeEval(gradGrid);

		// estimate center location - return order is
		// most likely location at front with decreasing likelihood following)
		std::vector<sig::QuadWgt> const sigQuadWgts
			{ edgeEval.sigQuadWeights(gradGrid.hwSize()) };

		img::Spot gotCenterSpot{};
		if (! sigQuadWgts.empty())
		{
			// use first spotWgt (one with highest weight) as 'best' estimate
			// NOTE: the simulation oversampling treats pixels as areas
			//       whereas the image processing treats them as point
			//       samples. Therefore, add {.5,.5} to match the simulation.
			gotCenterSpot = img::Spot
					{ sigQuadWgts.front().item().centerSpot()
					+ img::Spot{ .5, .5 }
					};
		}

		// [DoxyExample01]


/*
std::cout << pixGrid.infoStringContents("pixGrid", "%5.2f") << '\n';
(void)io::writeStretchPGM("pixGrid.pgm", pixGrid);

sig::GroupTable const groupTab{ edgeEval.groupTable() };
std::cout << groupTab.infoStringContents("groupTab", "%5.3f") << '\n';

std::vector<sig::EdgeGroup> const edgeGroups{ edgeEval.edgeGroups() };
std::vector<sig::RayWgt> const rayWgts
	{ edgeEval.groupRayWeights(edgeGroups) };
std::cout << infoStringFor(rayWgts, "t.rayWgts") << '\n';

ras::Grid<float> const eiGrid
	{ edgeEval.edgeInfoGrid(gradGrid.hwSize()) };
std::cout << eiGrid.infoStringContents("eiGrid", "%5.2f") << '\n';
(void)io::writeStretchPGM("edgeInfoMag.pgm", eiGrid);

std::cout << "sigQuadWgts.size: " << sigQuadWgts.size() << '\n';

std::ofstream ofsSpot("spot.dat");
for (sig::QuadWgt const & sigQuadWgt : sigQuadWgts)
{
	std::cout << sigQuadWgt << '\n';
	ofsSpot << "sigQuadWgt:"
		<< ' ' << sigQuadWgt.item().centerSpot()
		<< ' ' << engabra::g3::io::fixed(sigQuadWgt.weight())
		<< '\n';
}
*/

		std::vector<sig::AngleWgt> const peakAWs
			{ edgeEval.peakAngleWeights() };

		// should be four or more radial directions for simulated quad image
		if (! (3u < peakAWs.size()))
		{
			oss << "Failure of sufficient angle peak detection test\n";
			oss << "exp: (3u < peakAWs.size())\n";
			oss << "got: " << peakAWs.size() << '\n';
		}

		double const tolCenter{ .5 };
		img::Spot const difCenterSpot{ gotCenterSpot - expCenterSpot };
		double const difMag{ magnitude(difCenterSpot) };
		if (! nearlyEqualsAbs(gotCenterSpot, expCenterSpot, tolCenter))
		{
			oss << "Failure of gotCenterSpot test\n";
			oss << "exp: " << expCenterSpot << '\n';
			oss << "got: " << gotCenterSpot << '\n';
			oss << "dif: " << difCenterSpot << '\n';
			oss << "err: " << difMag << '\n';
			oss << "tol: " << tolCenter << '\n';
		}
		else
		{
			std::cout << "\n\n SUCCESS center test\n";
			std::cout << "exp: " << expCenterSpot << '\n';
			std::cout << "got: " << gotCenterSpot << '\n';
		//	std::cout << "dif: " << difCenterSpot << '\n';
			std::cout << "err: " << difMag << '\n';
		//	std::cout << "tol: " << tolCenter << '\n';
			std::cout << "\n";
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

