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
#include "cast.hpp"
#include "datRing.hpp"
#include "houghAdderAD.hpp"
#include "imgCamera.hpp"
#include "imgQuadTarget.hpp"
#include "io.hpp"
#include "objQuadTarget.hpp"
#include "pixEdgel.hpp"
#include "pixgrid.hpp"
#include "prbGauss1D.hpp"
#include "sigPeakFinder.hpp"
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

	//! Simulate images for demonstration
	inline
	std::vector<dat::Grid<float> >
	quadImages
		( std::size_t const & numImages
		)
	{
		std::vector<dat::Grid<float> > grids;
		grids.reserve(10u);

		// configure camera (including image size)
		static dat::SizeHW const hwGrid{ 128u, 128u };
		static double const pd{ 200. };
		static img::Camera const camera{ hwGrid, pd };

		// quad target size
		static obj::QuadTarget const objQuad
			( .125
			, obj::QuadTarget::AddSurround
			| obj::QuadTarget::DoubleTriangle
			);

		// simulate a collection with slightly different perspectives
		for (std::size_t nn{0u} ; nn < numImages ; ++nn)
		{
			// orientation of camera
			using namespace rigibra;
			using namespace engabra::g3;
			double dist{ .15 + .2*(double)(nn) };
std::cout << "dist: " << dist << '\n';
			double const da{ .125 * (double)nn };
			double const dx{ dist * da };
			Transform const xCamWrtQuad
				{ Vector{ dx, -dx, dist }
				, Attitude{ PhysAngle{ BiVector{ da, da, 0. } } }
				};

			sim::Render const render(camera, xCamWrtQuad, objQuad);
			std::cout << "render: " << render << '\n';

			// simulate pixel image
			grids.emplace_back(render.quadImage());
		}

		return grids;
	}

} // [sim]

namespace pix
{
	using namespace quadloco;

	//! Compute edge elements from gradient grid
	inline
	std::vector<pix::Edgel>
	edgelsFromGradGrid
		( dat::Grid<pix::Grad> const & gradGrid
		)
	{
		std::vector<pix::Edgel> pixEdgels{};
		pixEdgels.reserve(gradGrid.size());

		// could skip unset edge rows
		for (std::size_t row{0u} ; row < gradGrid.high() ; ++row)
		{
			for (std::size_t col{0u} ; col < gradGrid.wide() ; ++col)
			{
				pix::Grad const & grad = gradGrid(row, col);
				if (grad.isValid())
				{
					pix::Edgel const edgel(dat::RowCol{ row, col }, grad);
					pixEdgels.push_back(edgel);
				}
			}
		}

		return pixEdgels;
	}

	inline
	dat::Grid<float>
	edgeMagGridFor
		( dat::SizeHW const & hwSize
		, std::vector<pix::Edgel> const & pixEdgels
		, std::size_t const & numToUse
		)
	{
		dat::Grid<float> magGrid(hwSize);
		std::fill(magGrid.begin(), magGrid.end(), 0.f);
		for (std::size_t nn{0u} ; nn < numToUse ; ++nn)
		{
			pix::Edgel const & edgel = pixEdgels[nn];
			dat::RowCol const rc{ cast::datRowCol(edgel.location()) };
			magGrid(rc) = magnitude(edgel.gradient());
		}

		return magGrid;
	}


	//! Attributes of single Edgel
	struct EdgeInfo
	{
		pix::Edgel const theEdgel;
		pix::Spot const theSpot;
		pix::Grad const theGrad;
		double const theGradMag;
		pix::Grad const theGradDir;

		inline
		explicit
		EdgeInfo
			( pix::Edgel const & edgel
			)
			: theSpot{ edgel.location() }
			, theGrad{ edgel.gradient() }
			, theGradMag{ magnitude(theGrad) }
			, theGradDir{ (float)(1./theGradMag) * theGrad }
		{ }

		//! Angle of gradient direciont
		inline
		double
		angleOfGrad
			() const
		{
			return quadloco::ang::atan2(theGrad[1], theGrad[0]);
		}

	}; // EdgeInfo

	//! Edgel pair attributes for candidates on opposite radial edges
	class EdgePair
	{
		std::size_t theNdx1;
		std::size_t theNdx2;
		double theDotAlign; // negative dot product of edge gradients
		double theWgtFacing; // pseudo prop of oppositely directed edges
		double theLineGapDist; // distance between edge line segments
		double theWgtLineGap; // pseudo prop of edgels being collinear

		//! Negative of dot product between the two gradient edges
		inline
		static
		double
		dotGradDirs
			( std::size_t const & ndx1
			, std::size_t const & ndx2
			, std::vector<EdgeInfo> const & edgeInfos
			)
		{
			EdgeInfo const & ei1 = edgeInfos[ndx1];
			EdgeInfo const & ei2 = edgeInfos[ndx2];
			return dot(ei1.theGradDir, ei2.theGradDir);
		}

		//! Average distance of other edge location from own edge line.
		inline
		static
		double
		lineGapDist
			( std::size_t const & ndx1
			, std::size_t const & ndx2
			, std::vector<EdgeInfo> const & edgeInfos
			)
		{
			// separation of the two lines
			EdgeInfo const & ei1 = edgeInfos[ndx1];
			EdgeInfo const & ei2 = edgeInfos[ndx2];
			pix::Spot const & spot1 = ei1.theSpot;
			pix::Spot const & spot2 = ei2.theSpot;
			pix::Grad const & dir1 = ei1.theGradDir;
			pix::Grad const & dir2 = ei2.theGradDir;
			double const dist2from1{ dot((spot2 - spot1), dir1) };
			double const dist1from2{ dot((spot1 - spot2), dir2) };
			double const distBetween{ .5 * (dist2from1 + dist1from2) };
			return distBetween;
		}

		/*! \brief Pseudo-probability that edges are facing each other.
		 *
		 * (arbitrarily) use cos()^N as weighting function for which
		 * N=30 approximates exp(-(x/.25)^2) which is a Gaussian
		 * with (0.25 == sigma) or about +/- 15deg for 1-sigma, and
		 * +/- 45 deg for 3-sigma excursions
		 */
		inline
		static
		double
		weightFacing
			( double const & dotAlign
				//!< Negative of dot product of the two edgel gradients
			)
		{
			return std::pow(dotAlign, 30.);
		}

		//! Pseudo-probability of being on the same line given sigma
		inline
		static
		double
		weightLineGap
			( double const & lineGapDist
				//!< Separation of lines between two edgels
			, double const & lineGapSigma
				//!< Uncertainty in collinearity (tolerance is 3x this)
			, double const & theDotAlign
				//!< Relative (anti)alignment used to qualify computation
			, double const & theDotAlignMin = .75
			)
		{
			double wgt{ 0. };
			if (theDotAlignMin < theDotAlign)
			{
				double const lineGapMax{ 4. * lineGapSigma };
				if (lineGapDist < lineGapMax)
				{
					// assign pseudo weight on collinearity
					double const arg{ lineGapDist / lineGapSigma };
					double const wgtLineGap{ std::exp(-arg*arg) };
					wgt = wgtLineGap;
				}
			}
			return wgt;
		}

	public:

		inline
		explicit
		EdgePair
			( std::size_t const & ndx1
			, std::size_t const & ndx2
			, std::vector<EdgeInfo> const & edgeInfos
			, double const & lineGapSigma = 2.
			)
			: theNdx1{ ndx1 }
			, theNdx2{ ndx2 }
			, theDotAlign{ -dotGradDirs(theNdx1, theNdx2, edgeInfos) }
			, theWgtFacing{ weightFacing(theDotAlign) }
				// Average distance of other pixel to own edge line seg
			, theLineGapDist{ lineGapDist(theNdx1, theNdx2, edgeInfos) }
			, theWgtLineGap
				{ weightLineGap(theLineGapDist, lineGapSigma, theDotAlign) }
		{ }

		//! Pseudo-probability of edges facing each other
		inline
		double
		weightFacing
			() const
		{
			double wgt{ 0. };
			if (0. < theDotAlign)
			{
				wgt = theWgtFacing;
			}
			return wgt;
		}

		//! Pseudo-probability of being on the same line given sigma
		inline
		double const &
		weightLineGap
			() const
		{
			return theWgtLineGap;
		}

		/*! \brief Pseudo-probability edgels are on opposite radial edges.
		 *
		 * Assign pseudo probability of radial edge pair requiring:
		 * \arg Oppositing gradient directions (for +/- radius)
		 * \arg Approx collinearity of spots perp to edge grads
		 */
		inline
		double
		weightRadialPair
			() const
		{
			return (weightFacing() * weightLineGap());
		}

		//! Gradient associated with pair of edgels
		inline
		pix::Grad
		meanDir
			( std::vector<EdgeInfo> const & edgeInfos
			) const
		{
			EdgeInfo const & ei1 = edgeInfos[theNdx1];
			EdgeInfo const & ei2 = edgeInfos[theNdx2];
			pix::Grad const & dir1 = ei1.theGradDir;
			pix::Grad const & dir2 = ei2.theGradDir;
			//
			// NOTE: treat dir1 as positive direction and negate dir2
			//
			pix::Grad const sum{ dir1 - dir2 };
			float const mag{ magnitude(sum) };
			pix::Grad const mean{ (1.f/mag) * sum };
			return mean;
		}

		//! Angle associated with dir
		inline
		double
		angle
			( pix::Grad const & dir
			) const
		{
			return std::atan2(dir[1], dir[0]);
		}

	}; // EdgePair

} // [pix]

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
	std::vector<dat::Spot>
	spotLocsFor
		( std::vector<double> const & angles
		)
	{
		std::vector<dat::Spot> spotLocs;
		spotLocs.reserve(angles.size());
		for (double const & angle : angles)
		{
			dat::Spot const spotLoc{ std::cos(angle), std::sin(angle) };
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
		, std::vector<pix::EdgeInfo> const & edgeInfos
		, double const & angleSigma = 3.14*(10./180.)
		)
	{
		std::size_t const numEdges{ edgeInfos.size() };
		std::size_t const numGroups{ peakAngles.size() };

		std::vector<GroupMembers> groups(numGroups, GroupMembers{});

		// spot locations for each angle
		std::vector<dat::Spot> const spotLocs{ spotLocsFor(peakAngles) };

		// determine closest spot for each edge direction
		for (std::size_t ndxEdge{0u} ; ndxEdge < numEdges ; ++ndxEdge)
		{
			pix::EdgeInfo const & edgeInfo = edgeInfos[ndxEdge];
			pix::Grad const & gradDir = edgeInfo.theGradDir;

			// treat directions as spot locations (on unit circle)
			dat::Spot const gradSpot{ gradDir[0], gradDir[1] };

			// determine group closest to this edge direction
			double distMin{ 8. }; // much larger than unit circle diameter
			std::size_t ndxMin{ 0u };
			for (std::size_t ndxGroup{0u} ; ndxGroup < numGroups ; ++ndxGroup)
			{
				dat::Spot const & spotLoc = spotLocs[ndxGroup];
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
	std::vector<quadloco::dat::Grid<float> > const pixGrids
		{ quadloco::sim::quadImages(numImages) };

	// loop over sample images
	for (std::size_t nn{0u} ; nn < pixGrids.size() ; ++nn)
	{
		using namespace quadloco;

		dat::Grid<float> const & pixGrid = pixGrids[nn];

		// compute gradient at each pixel (interior to edge padding)
		dat::Grid<pix::Grad> const gradGrid
			{ pix::grid::gradientGridFor(pixGrid) };

		// assemble collection of edge elements from gradient grid
		std::vector<pix::Edgel> pixEdgels
			{ edgelsFromGradGrid(gradGrid) };

		// estimate a reasonable number of edge pixels to expect
		// (assume target nearly filling diagaonals and with 2 pix edges)
		double const diag{ pixGrid.hwSize().diagonal() };
		std::size_t const numToUse{ static_cast<std::size_t>(8. * diag) };

		// gather the strongest edges at start of collection
		std::vector<pix::Edgel>::iterator const iterToUse
			{ pixEdgels.begin() + numToUse };
		std::partial_sort
			( pixEdgels.begin(), iterToUse, pixEdgels.end()
			, [] (pix::Edgel const & e1, pix::Edgel const & e2)
				{ return magnitude(e2.gradient()) < magnitude(e1.gradient()) ; }
			);


		// Individual edge properties
		std::vector<pix::EdgeInfo> edgeInfos;
		edgeInfos.reserve(numToUse);

		// compute useful properties for individual edges
		for (std::size_t ndx{0u} ; ndx < numToUse ; ++ndx)
		{
			pix::Edgel const & edgel = pixEdgels[ndx];
			edgeInfos.emplace_back(pix::EdgeInfo(edgel));
		}


		// Candidate edgel pairs for opposite radial edges
		std::vector<pix::EdgePair> edgePairs;
		std::size_t const numWorstCase{ (numToUse * (numToUse - 1u)) / 2u };
		edgePairs.reserve(numWorstCase);

		// find pairs of edgels likely on opposite radial edges
		for (std::size_t ndx1{0u} ; ndx1 < numToUse ; ++ndx1)
		{
			for (std::size_t ndx2{ndx1+1} ; ndx2 < numToUse ; ++ndx2)
			{
				// remember the interesting edges
				pix::EdgePair const edgePair(ndx1, ndx2, edgeInfos);

				// assign pseudo probability of radial edge pair
				// which requires:
				// - oppositing gradient directions (for +/- radius)
				// - approx collinearity of spots perp to edge grads
				double const wgtRadial{ edgePair.weightRadialPair() };

				// remember pairs which are likely to be on opposite edges
				if (.75 < wgtRadial)
				{
					edgePairs.emplace_back(edgePair);
				}
			}
		}

		// create angle value accumulation (circular-wrapping) buffer
		std::size_t const numAngBins{ 32u };
		ang::Likely angleProbs(numAngBins);
		for (pix::EdgePair const & edgePair : edgePairs)
		{
			pix::Grad const meanDir{ edgePair.meanDir(edgeInfos) };
			double const angle{ edgePair.angle(meanDir) };
			double const weight{ edgePair.weightRadialPair() };
			angleProbs.add(angle, weight, 2);
		}

		// get peaks from angular accumulation buffer
		std::vector<double> const peakAngles{ angleProbs.anglesOfPeaks() };

		struct LineGrouper
		{
			std::vector<pix::EdgeInfo> * const ptEdgeInfos{ nullptr };
			std::vector<double> * const ptPeakAngles{ nullptr };

		}; // Foo

		// classify original edgels by proximity to peak angles
		std::vector<tmp::GroupMembers> const lineGroups
			{ tmp::lineIndexGroups(peakAngles, edgeInfos) };

std::cout << '\n';
std::cout << angleProbs.infoStringContents("angleProbs") << '\n';
std::cout << angleProbs.infoString("angleProbs") << '\n';
std::cout << "numWorstCase: " << numWorstCase << '\n';
std::cout << "     numUsed: " << edgePairs.size() << '\n';
for (double const & peakAngle : peakAngles)
{
	std::cout << "peakAngle: " << engabra::g3::io::fixed(peakAngle) << '\n';
}

std::ofstream ofsGroup("./foo.dat");
for (std::size_t ndxGroup{0u} ; ndxGroup < lineGroups.size() ; ++ndxGroup)
{
	tmp::GroupMembers const & lineGroup = lineGroups[ndxGroup];

	ofsGroup << "\n\n";
	for (tmp::Member const & member : lineGroup)
	{
		std::size_t const & ndxEdge = member.theNdxEdge;
		double const & prob = member.theProb;
		pix::EdgeInfo const & ei = edgeInfos[ndxEdge];
		ofsGroup
			<< "edgeSpot,Group:"
			<< ' ' << ei.theSpot
			<< ' ' << engabra::g3::io::fixed(prob)
			<< ' ' << ndxGroup
			<< '\n';
			;

		/* EdgeInfo
		pix::Edgel const theEdgel;
		pix::Spot const theSpot;
		pix::Grad const theGrad;
		double const theGradMag;
		pix::Grad const theGradDir;
		*/
	}
}

		//
		// Write edge pair data to file for diagnostic use
		//

		std::string const datName{ std::format("./sample{:02d}dat.pgm", nn) };
		std::string const pixName{ std::format("./sample{:02d}pix.pgm", nn) };
		std::string const magName{ std::format("./sample{:02d}mag.pgm", nn) };

		std::ofstream ofs(datName);
		ofs << "#\n";
		ofs << "# MeanDir[0,1] angle wgtFacing wgtLineGab wgtRadialPair\n";
		ofs << "#\n";

		for (pix::EdgePair const & edgePair : edgePairs)
		{
			pix::Grad const meanDir{ edgePair.meanDir(edgeInfos) };
			using engabra::g3::io::fixed;
			ofs
				<< ' ' << meanDir                            // 1,2
				<< ' ' << fixed(edgePair.angle(meanDir))     // 3
				<< ' ' << fixed(edgePair.weightFacing())     // 4
				<< ' ' << fixed(edgePair.weightLineGap())    // 5
				<< ' ' << fixed(edgePair.weightRadialPair()) // 6
				<< '\n';
		}


		// draw strongest pixels into grid (for development feedback)
		dat::Grid<float> const magGrid
			(edgeMagGridFor(pixGrid.hwSize(), pixEdgels, numToUse));

		io::writeStretchPGM(pixName, pixGrid);
		io::writeStretchPGM(magName, magGrid);

		std::cout << '\n';
		std::cout << "pixEdgels.size: " << pixEdgels.size() << '\n';
		std::cout << "Linedata written to '" << datName << "'\n";
		std::cout << "Pix data written to '" << pixName << "'\n";
		std::cout << "Mag data written to '" << magName << "'\n";

break;

	}
	std::cout << '\n';

	return 0;
}


