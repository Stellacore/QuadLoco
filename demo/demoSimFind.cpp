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
\brief Experiment at locating center
*/


#include "imgSpot.hpp"
#include "objCamera.hpp"
#include "objQuadTarget.hpp"
#include "opsgrid.hpp"
#include "rasGrid.hpp"
#include "sigEdgeEval.hpp"
#include "sigItemWgt.hpp"
#include "sigQuadTarget.hpp"
#include "simRender.hpp"

#include <Engabra>
#include <Rigibra>

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>



namespace quadloco
{
namespace sim
{
	using namespace quadloco;

	//! Simulated raster data and geometric image shape
	struct TestCase
	{
		ras::Grid<float> thePixGrid;
		sig::QuadTarget theSigQuad;

	}; // TestCase

	//! Center point comparison
	struct TestResult
	{
		std::size_t const theNdx;
		sig::SpotWgt const theExpSW{};
		sig::SpotWgt const theGotSW{};

		//! Descriptive information about this instance.
		inline
		std::string
		infoString
			( std::string const & title = {}
			) const
		{
			std::ostringstream oss;
			if (! title.empty())
			{
				oss << title << ' ';
			}
			img::Spot const & expSpot = theExpSW.item();
			img::Spot const & gotSpot = theGotSW.item();
			img::Spot const difSpot{ gotSpot - expSpot };
			using engabra::g3::io::fixed;
			oss
				<< "ndx: " << std::setw(3u) << theNdx
				<< "  "
				<< "difSpot: " << difSpot
				<< "  "
				<< "expSpot: " << expSpot
				<< "  "
				<< "gotSpot: " << gotSpot
				<< "  wgt: " << fixed(theGotSW.weight())
				;

			return oss.str();
		}

	}; // TestResult


	//! Simulate images for demonstration
	inline
	std::vector<TestCase>
	testCases
		( std::size_t const & numImages
		)
	{
		std::vector<TestCase> cases;
		cases.reserve(10u);

		// configure camera (including image size)
		constexpr ras::SizeHW hwGrid{ 128u, 128u };
		constexpr double pd{ 200. };
		static obj::Camera const camera{ hwGrid, pd };

		// quad target size
		static obj::QuadTarget const objQuad
			( .125
			, obj::QuadTarget::WithSurround
			| obj::QuadTarget::WithTriangle
			);

		// simulate a collection with slightly different perspectives
		for (std::size_t nn{0u} ; nn < numImages ; ++nn)
		{
			// orientation of camera
			using namespace rigibra;
			using namespace engabra::g3;
			double dist{ .15 + .2*(double)(nn) };
			double const da{ .125 * (double)nn };
			double const dx{ dist * da };
			Transform const xCamWrtQuad
				{ Vector{ dx, -dx, dist }
				, Attitude{ PhysAngle{ BiVector{ da, da, 0. } } }
				};

			sim::Render const render(camera, xCamWrtQuad, objQuad);
			std::cout << "render: " << render << '\n';

			// simulate pixel image
			cases.emplace_back
				(TestCase{ render.quadImage(), render.sigQuadTarget() });
		}

		return cases;
	}

} // [sim]

} // [quadloco]



/*! \brief Find center location in simulated quad images
*/
int
main
	()
{
	constexpr std::size_t numImages{ 7u };
	std::vector<quadloco::sim::TestCase> const testCases
		{ quadloco::sim::testCases(numImages) };
	std::vector<quadloco::sim::TestResult> testResults{};
	testResults.reserve(testCases.size());

	// loop over sample images
	for (std::size_t nn{0u} ; nn < testCases.size() ; ++nn)
	{
		using namespace quadloco;

		sim::TestCase const & testCase = testCases[nn];
		ras::Grid<float> const & pixGrid = testCase.thePixGrid;

		// compute gradient at each pixel (interior to edge padding)
		ras::Grid<img::Grad> const gradGrid
			{ ops::grid::gradientGridFor(pixGrid) };

		// evaluate gradients...
		sig::EdgeEval const edgeEval(gradGrid);
		// ... and determine candidate quad images
		std::vector<sig::QuadWgt> const sigQuadWgts
			{ edgeEval.sigQuadWeights(gradGrid.hwSize()) };

		// add to test result
		sig::QuadTarget const & expQuad = testCase.theSigQuad;;
		sig::SpotWgt const expSW{ expQuad.centerSpot(), 1. };
		for (sig::QuadWgt const & sigQuadWgt : sigQuadWgts)
		{
			if (isValid(sigQuadWgt))
			{
				sig::QuadTarget const & sigQuad = sigQuadWgt.item();
				double const & wgt = sigQuadWgt.weight();
				img::Spot const gotSpot{ sigQuad.centerSpot() };
				sig::SpotWgt const gotSW{ gotSpot, wgt };

				sim::TestResult const testResult{ nn, expSW, gotSW };
				testResults.emplace_back(testResult);
			}
		}

	} // testCases

	std::cout << '\n';

	// display results
	std::cout << "\n\ntestResults: " << testResults.size() << '\n';
	std::size_t ndxPrev{ 0u };
	for (quadloco::sim::TestResult const & testResult : testResults)
	{
		if (testResult.theNdx != ndxPrev)
		{
			std::cout << '\n';
			ndxPrev = testResult.theNdx;
		}
		std::cout << "testResult: " << testResult.infoString() << '\n';
	}

	return 0;
}


