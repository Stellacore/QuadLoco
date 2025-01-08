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


#include "sigedgel.hpp"

#include "imgEdgel.hpp"
#include "io.hpp"
#include "objCamera.hpp"
#include "objQuadTarget.hpp"
#include "opsgrid.hpp"
#include "opsPeakFinder1D.hpp"
#include "prbHisto.hpp"
#include "sigEdgeEval.hpp"
#include "sigedgel.hpp"
#include "sigutil.hpp"
#include "simConfig.hpp"
#include "simRender.hpp"

#include <Engabra>
#include <Rigibra>

#include <fstream>
#include <iostream>
#include <sstream>


namespace
{
	inline
	quadloco::obj::Camera
	cameraFitTo
		( std::size_t const & numPixOnEdge
		)
	{
		return quadloco::obj::Camera
			{ quadloco::ras::SizeHW{ numPixOnEdge, numPixOnEdge }
			, static_cast<double>(numPixOnEdge) // pd
			};
	}

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
//			, cameraFitTo(128u)
//			, cameraFitTo( 32u)
//			, cameraFitTo( 31u)
			, cameraFitTo( 16u)
//			, cameraFitTo( 15u)
//			, cameraFitTo(  8u)
//			, cameraFitTo(  7u)
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
		quadloco::img::QuadTarget imgQuad{};  // set by simulation
		ras::Grid<float> const srcGrid{ simulatedQuadGrid(&imgQuad) };
		ras::Grid<float> const & pixGrid = srcGrid;
//		ras::Grid<float> const pixGrid
//			{ ops::grid::smoothGridFor<float>(srcGrid, 1u, 0.75) };
		img::Spot const expCenterSpot{ imgQuad.centerSpot() };

		// compute gradient elements
		ras::Grid<img::Grad> const gradGrid
			{ ops::grid::gradientGridBy8x(pixGrid) };

		// categorize edgels as candidates for (radial) quad target edge groups
		sig::EdgeEval const edgeEval(gradGrid);

		// estimate center location - return order is
		// most likely location at front with decreasing likelihood following)
		std::vector<sig::QuadWgt> const imgQuadWgts
			{ edgeEval.imgQuadWeights(gradGrid.hwSize()) };

		img::Spot gotCenterSpot{};
		double gotCenterSigma{ std::numeric_limits<double>::quiet_NaN() };
		if (! imgQuadWgts.empty())
		{
			// use first spotWgt (one with highest weight) as 'best' estimate
			// NOTE: the simulation oversampling treats pixels as areas
			//       whereas the image processing treats them as point
			//       samples. Therefore, add {.5,.5} to match the simulation.
			gotCenterSpot = img::Spot
					{ imgQuadWgts.front().item().centerSpot()
					};
			gotCenterSigma = imgQuadWgts.front().item().centerSigma();
		}

		// [DoxyExample01]

//
// === Diagnostics
//

	// Extract strongest gradient edges
	std::vector<img::Edgel> const corrEdgels
		{ sig::edgel::dominantEdgelsFrom(gradGrid, 2.5, 4u) };
	std::ofstream ofsDEs("edgeDom.dat");
	for (img::Edgel const & corrEdgel : corrEdgels)
	{
		ofsDEs << corrEdgel << '\n';
	}

	// Assign weights to these based on degree of radial corroberation
	std::vector<sig::EdgeInfo> const edgeInfos
		{ sig::edgel::edgeInfosLikelyRadial(corrEdgels) };
	std::ofstream ofsEIs("edgeInfos.dat");
	for (sig::EdgeInfo const & edgeInfo : edgeInfos)
	{
		using engabra::g3::io::fixed;
		ofsEIs
			<< "location: " << edgeInfo.edgeLocation()
			<< " considerDir: " << edgeInfo.consideredDirection()
			<< " wgt: " << fixed(edgeInfo.consideredWeight())
			<< '\n';
	}

// TODO - angles peaks are duplicated (because of symmetry accumulation)
//        can be optimized (ideally by peak accumulation in half size array)
	// Estimate dominant edge direction angles
	std::size_t const numAngBins{ gradGrid.hwSize().perimeter() };
	std::vector<sig::AngleWgt> const angWgts
		{ sig::EdgeGrouper::peakAngleWeights(edgeInfos, numAngBins) };

	// loop over peaks in gradient direction angle
	// for each one, determine peaks in edgel locations along that direction
std::size_t lineCount{ 0u };
	for (sig::AngleWgt const & angWgt : angWgts)
	{
		double const & angle = angWgt.item();
		double const & wgt = angWgt.weight();
		img::Vector<double> const angleDir{ cos(angle), sin(angle) };

		double const diag{ gradGrid.hwSize().diagonal() };
		std::size_t const numDistBins{ (std::size_t)diag };
		img::Span const distSpan{ -diag, diag };
		prb::Histo distHist(4u*numDistBins, distSpan);
		for (sig::EdgeInfo const & edgeInfo : edgeInfos)
		{
			img::Vector<double> const & loc = edgeInfo.edgeLocation();
			double const proj{ dot(loc, angleDir) };
			constexpr double sigma{ 2. };
			distHist.addValue(proj, sigma);
		}

		std::vector<double> const & binProbs = distHist.probabilities();
		ops::PeakFinder1D const peakFinder
			(binProbs.cbegin(), binProbs.cend(), ops::PeakFinder1D::Linear);
		std::vector<std::size_t> const peakNdxs{ peakFinder.peakIndices() };

		std::cout << '\n';
		std::cout << "angWgt: " << angWgt << '\n';
		// std::cout << '\n';
		// std::cout << distHist.infoStringContents("distHist") << '\n';
		for (std::size_t const & peakNdx : peakNdxs)
		{
			std::cout << "peak:"
				<< " ndx: " << std::setw(3u) << peakNdx
				<< " val: " << std::setw(9u) << std::fixed
					<< distHist.valueAtIndex(peakNdx)
				<< " prob: " << std::setw(9u) << std::fixed
					<< distHist.probabilityAtIndex(peakNdx)
				<< '\n';

			double const distProb{ distHist.probabilityAtIndex(peakNdx) };

			double const dist{ distHist.valueAtIndex(peakNdx) };
			img::Spot const start{ dist * angleDir };
			img::Spot const lineDir{ ccwPerp(angleDir) };
			img::Ray const lineRay{ start, lineDir };

			std::cout << "lineRay: " << lineRay << '\n';

			std::ostringstream msg;
			msg << "line" << (100u + lineCount++) << ".dat";
			std::ofstream ofsLine(msg.str());
			ofsLine << "# Row, Col, Wgt, Rejection\n";
			for (sig::EdgeInfo const & edgeInfo : edgeInfos)
			{
				img::Vector<double> const & loc = edgeInfo.edgeLocation();
				img::Vector<double> const lineDir
					{ lineRay.direction() };
				img::Vector<double> const edgeDir
					{ edgeInfo.edgel().direction() };
				double const align{ outer(lineDir, edgeDir) };

				double const rej{ lineRay.distanceFrom(loc) };
				double const sigma{ 1. };
				double const arg{ rej / sigma };
				double const wgt{ distProb * align * std::exp(-arg*arg) };

				ofsLine
					<< loc
					<< ' '
					<< std::setw(12u) << std::fixed << wgt
					<< ' '
					<< std::setw(12u) << std::fixed << rej
					<< '\n';
				/*
					<< "loc: " << loc
					<< ' '
					<< "rej: " << std::setw(12u) << std::fixed << rej
					<< ' '
					<< "wgt: " << std::setw(12u) << std::fixed << wgt
					<< '\n';
				*/
			}

		}

	}


	std::ofstream ofsAWs("angWgts.dat");
	for (sig::AngleWgt const & angWgt : angWgts)
	{
		ofsAWs << "angWgt: " << angWgt << '\n';
	}


//
// === Save data to files
//

// pixels
(void)io::writeStretchPGM("pixGrid.pgm", pixGrid);

// gradient values
ras::Grid<float> const gradRow
	{ sig::util::rowCompGridFor(gradGrid) };
ras::Grid<float> const gradCol
	{ sig::util::colCompGridFor(gradGrid) };
(void)io::writeStretchPGM("gradRow.pgm", gradRow);
(void)io::writeStretchPGM("gradCol.pgm", gradCol);
ras::Grid<float> const gradMag
	{ sig::util::magnitudeGridFor(gradGrid) };
ras::Grid<float> const gradAng
	{ sig::util::angleGridFor(gradGrid) };
(void)io::writeStretchPGM("gradMag.pgm", gradMag);
(void)io::writeStretchPGM("gradAng.pgm", gradAng);

// EdgeInfo-mag
ras::Grid<float> const edgeInfoWeightGrid
	{ sig::util::edgeInfoWeightGrid
		(gradGrid.hwSize(), edgeEval.edgeInfos())
	};
(void)io::writeStretchPGM("edgeInfoWgt.pgm", edgeInfoWeightGrid);
ras::Grid<float> const edgeInfoAngleGrid
	{ sig::util::edgeInfoAngleGrid
		(gradGrid.hwSize(), edgeEval.edgeInfos())
	};
(void)io::writeStretchPGM("edgeInfoAng.pgm", edgeInfoAngleGrid);


// Edge ray data
std::ofstream ofsRay("ray.dat");
/*
std::vector<sig::EdgeGroupNdxWgts> const edgeGroups{ edgeEval.edgeGroups() };
std::vector<sig::RayWgt> const rayWgts{ edgeEval.groupRayWeights(edgeGroups) };
ofsRay << infoStringFor(rayWgts, "t.rayWgts") << '\n';
*/

std::ofstream ofsSpot("spot.dat");
for (sig::QuadWgt const & imgQuadWgt : imgQuadWgts)
{
	ofsSpot << "imgQuadWgt:"
		<< ' ' << imgQuadWgt.item().centerSpot()
		<< ' ' << engabra::g3::io::fixed(imgQuadWgt.weight())
		<< '\n';
}


/*
std::cout << pixGrid.infoStringContents("pixGrid", "%5.2f") << '\n';

sig::GroupTable const groupTab{ edgeEval.groupTable() };
std::cout << groupTab.infoStringContents("groupTab", "%5.3f") << '\n';
std::cout << infoStringFor(imgQuadWgts, "t.imgQuadWgt") << '\n';
*/
std::ofstream ofsEdgeInfo("edgeInfoMag.dat");
ofsEdgeInfo << edgeInfoWeightGrid.infoStringContents
	("# edgeInfoWeightGrid", "%15.12f") << '\n';
ofsEdgeInfo << edgeInfoAngleGrid.infoStringContents
	("# edgeInfoAngleGrid", "%15.12f") << '\n';

		double const tolCenter{ .5 };
		img::Spot const difCenterSpot{ gotCenterSpot - expCenterSpot };
		double const difMag{ magnitude(difCenterSpot) };
		if (! nearlyEqualsAbs(gotCenterSpot, expCenterSpot, tolCenter))
		{
			oss << "Failure of gotCenterSpot test\n";
			oss << "exp: " << expCenterSpot << '\n';
			oss << "got: " << gotCenterSpot
				<< "   sigma: " << gotCenterSigma << '\n';
			oss << "dif: " << difCenterSpot << '\n';
			oss << "err: " << difMag << '\n';
			oss << "tol: " << tolCenter << '\n';
		}
		else
		{
			constexpr char pad[] = "    ";
			std::cout << "\n\n:-) Successful center test\n";
			std::cout << pad << "exp: " << expCenterSpot << '\n';
			std::cout << pad << "got: " << gotCenterSpot
				<< "   sigma: " << gotCenterSigma << '\n';
		//	std::cout << pad << "dif: " << difCenterSpot << '\n';
			std::cout << pad << "err: " << difMag << '\n';
		//	std::cout << pad << "tol: " << tolCenter << '\n';
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

