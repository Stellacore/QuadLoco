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


#include "ang.hpp"
#include "angLikely.hpp"
#include "angRing.hpp"
#include "cast.hpp"
#include "imgEdgel.hpp"
#include "io.hpp"
#include "objCamera.hpp"
#include "objQuadTarget.hpp"
#include "opsAdderAD.hpp"
#include "opsgrid.hpp"
#include "opsPeakFinder.hpp"
#include "prbGauss1D.hpp"
#include "sigEdgeEval.hpp"
#include "sigEdgeInfo.hpp"
#include "sigQuadTarget.hpp"
#include "simRender.hpp"

#include <Engabra>
#include <Rigibra>

#include <algorithm>
#include <format>
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


namespace tmp
{
	using namespace quadloco;

	//! Membership of Edgel into an implicit group
	struct Member
	{
		//! Index into collection of edgels
		std::size_t theNdxEdge{ 0u };

		//! Pseudo-probablility of edge belonging to this group
		double theProb{ 0. };
	};

	using GroupMembers = std::vector<Member>;

	//! Spot locations for each angle
	inline
	std::vector<img::Spot>
	unitCircleSpotsFor
		( std::vector<double> const & angles
		)
	{
		std::vector<img::Spot> spotLocs;
		spotLocs.reserve(angles.size());
		for (double const & angle : angles)
		{
			img::Spot const spotLoc{ std::cos(angle), std::sin(angle) };
			spotLocs.emplace_back(spotLoc);
		}
		return spotLocs;
	}


	/*! Groups (of edgeInfo indices) corresponding with peakAngle indices.
	 *
	 * Each element of the return vectors corresponds with peakAngle indices:
	 * \arg returnValue.size() == peakAngles.size()
	 *
	 * The indices within each group, are indices into edgeInfos vector.
	 */
	inline
	std::vector<GroupMembers>
	lineIndexGroups
		( std::vector<double> const & peakAngles
		, std::vector<sig::EdgeInfo> const & edgeInfos
		, double const & angleSigma = 3.14*(10./180.)
		)
	{
		std::vector<GroupMembers> groups{};

		std::size_t const numEdges{ edgeInfos.size() };
		std::size_t const numGroups{ peakAngles.size() };

		if (0u < numGroups)
		{
			groups = std::vector<GroupMembers>(numGroups, GroupMembers{});

			// unit circle spot locations for each angle
			std::vector<img::Spot> const spotLocs
				{ unitCircleSpotsFor(peakAngles) };

			// determine closest spot for each edge direction
			for (std::size_t ndxEdge{0u} ; ndxEdge < numEdges ; ++ndxEdge)
			{
				sig::EdgeInfo const & edgeInfo = edgeInfos[ndxEdge];
constexpr double minWeight{ 1./1024./1024. };
				if (minWeight < edgeInfo.consideredWeight())
				{
					img::Grad const gradDir{ edgeInfo.edgeDirection() };

					// treat directions as spot locations (on unit circle)
					img::Spot const gradSpot{ gradDir[0], gradDir[1] };

					// determine group closest to current edge direction
					double distMin{ 8. }; // much larger than unit circle
					std::size_t ndxMin{ 0u };
					for (std::size_t
						ndxGroup{0u} ; ndxGroup < numGroups ; ++ndxGroup)
					{
						img::Spot const & spotLoc = spotLocs[ndxGroup];
						double const dist{ magnitude(gradSpot - spotLoc) };
						if (dist < distMin)
						{
							distMin = dist;
							ndxMin = ndxGroup;
						}
					}

					// add edge index to closest group
					double const arg{ distMin / angleSigma };
					double const prob{ std::exp(-arg*arg) };
					groups[ndxMin].emplace_back(tmp::Member{ndxEdge, prob});
				}
			}
		}

		return groups;
	}

} // [tmp]


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


		//
		// Extract center point and compare with test case known value
		//

		// Construct edge evaluator to work on gradient grid
		sig::EdgeEval const edgeEval(gradGrid);

/*
		// Fetch best estimated spot locations
		std::vector<sig::SpotWgt> const spotWgts
			{ edgeEval.spotWeightsOverall(gradGrid.hwSize()) };

		if (! spotWgts.empty())
		{
			img::Spot const expSpot{ testCase.theSigQuad.centerSpot() };
			double const expSigma{ 0. }; // exact
			sig::SpotWgt const expSW{ expSpot, expSigma };

			sig::SpotWgt const & gotSW = spotWgts.front();
			//g::Spot const & gotSpot = gotSW.item();
			//uble const & gotSigma = gotSW.weight();

			sim::TestResult const testResult{ nn, expSW, gotSW };
			testResults.emplace_back(testResult);
		}
*/


		//
		// Generate various data for inspection
		//

		// assemble collection of edge elements from gradient grid
		std::vector<img::Edgel> pixEdgels
			{ ops::grid::linkedEdgelsFrom(gradGrid) };

		// estimate a reasonable number of edge pixels to expect
		// (assume target nearly filling diagaonals and with 2 pix edges)
		double const diag{ pixGrid.hwSize().diagonal() };
		std::size_t const numToUse{ static_cast<std::size_t>(8. * diag) };

		// gather the strongest edges at start of collection
		std::vector<img::Edgel>::iterator const iterToUse
			{ pixEdgels.begin() + numToUse };
		std::partial_sort
			( pixEdgels.begin(), iterToUse, pixEdgels.end()
			, [] (img::Edgel const & e1, img::Edgel const & e2)
				{ return e2.magnitude() < e1.magnitude() ; }
			);

		// Individual edge properties
		std::vector<sig::EdgeInfo> edgeInfos;
		edgeInfos.reserve(numToUse);

		// compute useful properties for individual edges
		for (std::size_t ndx{0u} ; ndx < numToUse ; ++ndx)
		{
			img::Edgel const & edgel = pixEdgels[ndx];
			edgeInfos.emplace_back(sig::EdgeInfo(edgel));
		}

		std::size_t const numWorstCase{ (numToUse * (numToUse - 1u)) / 2u };

		// find pairs of edgels likely on opposite radial edges
		for (std::size_t ndx1{0u} ; ndx1 < numToUse ; ++ndx1)
		{
			sig::EdgeInfo & edgeInfo1 = edgeInfos[ndx1];

			for (std::size_t ndx2{ndx1+1} ; ndx2 < numToUse ; ++ndx2)
			{
				sig::EdgeInfo & edgeInfo2 = edgeInfos[ndx2];

				edgeInfo1.consider(edgeInfo2.edgel());
				edgeInfo2.consider(edgeInfo1.edgel());
			}
		}

		// create angle value accumulation (circular-wrapping) buffer
		std::size_t const numAngBins{ 32u };
		ang::Likely angleProbs(numAngBins);
		for (sig::EdgeInfo const & edgeInfo : edgeInfos)
		{
			double const angle{ edgeInfo.consideredAngle() };
			double const weight{ edgeInfo.consideredWeight() };
			angleProbs.add(angle, weight, 2);
		}

		// get peaks from angular accumulation buffer
		std::vector<double> const peakAngles{ angleProbs.anglesOfPeaks() };


		// classify original edgels by proximity to peak angles
		std::vector<tmp::GroupMembers> const lineGroups
			{ tmp::lineIndexGroups(peakAngles, edgeInfos) };

		//
		// Display various values and results
		//

		// Angle buffer
		std::cout << '\n';
		std::cout << angleProbs.infoStringContents("angleProbs") << '\n';
		std::cout << angleProbs.infoString("angleProbs") << '\n';
		std::cout << "numWorstCase: " << numWorstCase << '\n';
		std::cout << "    numToUse: " << numToUse << '\n';
		for (double const & peakAngle : peakAngles)
		{
			std::cout << "peakAngle: "
				<< engabra::g3::io::fixed(peakAngle) << '\n';
		}

		//
		// Write edge pair data to file for diagnostic use
		//

		std::string const grpName{ std::format("./sample{:02d}grp.txt", nn) };
		std::string const datName{ std::format("./sample{:02d}dat.txt", nn) };
		std::string const pixName{ std::format("./sample{:02d}pix.pgm", nn) };
		std::string const magName{ std::format("./sample{:02d}mag.pgm", nn) };
		std::string const angName{ std::format("./sample{:02d}ang.pgm", nn) };

		std::ofstream ofsGroup(grpName);
		for (std::size_t
			ndxGroup{0u} ; ndxGroup < lineGroups.size() ; ++ndxGroup)
		{
			tmp::GroupMembers const & lineGroup = lineGroups[ndxGroup];
			ofsGroup << "\n\n";
			for (tmp::Member const & member : lineGroup)
			{
				std::size_t const & ndxEdge = member.theNdxEdge;
				double const & classMemberProb = member.theProb;
				sig::EdgeInfo const & ei = edgeInfos[ndxEdge];
				using engabra::g3::io::fixed;
				ofsGroup
					<< "edge:Spot: " << ei.edgeLocation()
					<< ' '
					<< "classProb: " << fixed(classMemberProb)
					<< ' '
					<< "radialWeight: " << fixed(ei.consideredWeight())
					<< ' '
					<< "groupId: " << ndxGroup
					<< '\n';
					;
			}
		}

		std::ofstream ofs(datName);
		ofs << "#\n";
		ofs << "# MeanDir[0,1] angle wgtFacing wgtLineGab wgtRadialPair\n";
		ofs << "#\n";

		for (sig::EdgeInfo const & edgeInfo : edgeInfos)
		{
			using engabra::g3::io::fixed;
			ofs
				<< ' ' << edgeInfo.consideredDirection()     // 1,2
				<< ' ' << fixed(edgeInfo.consideredAngle())  // 3
				<< ' ' << fixed(edgeInfo.consideredWeight())           // 4
				<< '\n';
		}

		// draw strongest edgel magnitudes into grid (for development feedback)
		ras::Grid<float> const magGrid
			{ ops::grid::edgeMagGridFor
				(pixGrid.hwSize(), pixEdgels, numToUse)
			};

		// draw strongest edgel angles into grid (for dev)
		ras::Grid<float> const angGrid
			{ ops::grid::edgeAngleGridFor
				(pixGrid.hwSize(), pixEdgels, numToUse)
			};

		io::writeStretchPGM(pixName, pixGrid);
		io::writeStretchPGM(magName, magGrid);
		io::writeStretchPGM(angName, angGrid);

		std::cout << '\n';
		std::cout << "pixEdgels.size: " << pixEdgels.size() << '\n';
		std::cout << "Linedata written to '" << datName << "'\n";
		std::cout << "Pix data written to '" << pixName << "'\n";
		std::cout << "Mag data written to '" << magName << "'\n";
		std::cout << "Ang data written to '" << angName << "'\n";

//break;

	}
	std::cout << '\n';

	// display results
	std::cout << "\n\ntestResults: " << testResults.size() << '\n';
	for (quadloco::sim::TestResult const & testResult : testResults)
	{
		std::cout << "testResult: " << testResult.infoString() << '\n';
	}

	return 0;
}


