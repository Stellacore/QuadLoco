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
#include "objCamera.hpp"
#include "objQuadTarget.hpp"
#include "opsgrid.hpp"
#include "sigEdgeEval.hpp"
#include "simConfig.hpp"
#include "simRender.hpp"

#include <Engabra>
#include <Rigibra>

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
		sim::Config const config
			{ obj::QuadTarget
				( .25
				, obj::QuadTarget::AddSurround
				)
			, obj::Camera
//				{ ras::SizeHW{ 128u, 128u }
{ ras::SizeHW{ 8u, 8u }
				, 100. // pd
				}
			, rigibra::Transform
				{ engabra::g3::Vector{ 0., 0., 1. }
				, rigibra::Attitude
					{ rigibra::PhysAngle
						{ engabra::g3::BiVector{ 0., 0., 0. } }
					}
				}
			};
		sim::Render const render
			( config
		//	, sim::Sampler::None
			, sim::Sampler::UseSceneBias
			| sim::Sampler::UseImageNoise
			);
		std::size_t const numOverSample{ 128u };
		if (ptSigQuad)
		{
			*ptSigQuad = render.imgQuadTarget();
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
		ras::Grid<float> const pixGrid{ simulatedQuadGrid() };
std::cout << pixGrid.infoStringContents("pixGrid", "%5.2f") << '\n';

		// compute gradient elements
		ras::Grid<img::Grad> const gradGrid
			{ ops::grid::gradientGridFor(pixGrid) };
		// assess significant gradients in context with other
		std::vector<img::Edgel> const edgels
			{ ops::grid::linkedEdgelsFrom(gradGrid) };

		// categorize edgels into counter directed groups
		sig::EdgeEval const edgeEval(gradGrid);

		std::vector<double> const peakAngles
			{ edgeEval.peakAngles() };


/*
		// estimate center from grouped edgels
		std::vector<img::Spot> const gotCenters{ groups.estimatedCenter() };
		img::Spot const gotCenter{ groups.center() };
*/

		// [DoxyExample01]

		// should be four or more radial directions for simulated quad image
		if (! (3u < peakAngles.size()))
		{
			oss << "Failure of sufficient angle peak detection test\n";
			oss << "exp: (3u < peakAngles.size())\n";
			oss << "got: " << peakAngles.size() << '\n';
		}

		// TODO replace this with real test code
		std::string const fname(__FILE__);
		bool const isTemplate{ (std::string::npos != fname.find("/_.cpp")) };
		if (! isTemplate)
		{
			oss << "Failure to implement real test\n";
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

