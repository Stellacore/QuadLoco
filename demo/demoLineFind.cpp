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

		// find the strongest edges
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

		//! Accumulation buffer
		struct AngleProbs
		{
			dat::Ring const theRing{};
			std::vector<double> theBinSums{};

			inline
			explicit
			AngleProbs
				( std::size_t const & numAngBins
				)
				: theRing(numAngBins)
				, theBinSums(numAngBins, 0.)
			{ }

			//! Number of bins in accumulation buffer.
			inline
			std::size_t
			size
				() const
			{
				return theBinSums.size();
			}

			//! Incorporate weighted angle value into ring buffer
			inline
			void
			add
				( double const & angle
					//!< Angle value at which to add weighted contributions
				, double const & weight = 1.
					//!< Overall weight with which to accumulate this angle
				, std::size_t const & halfBinSpread = 1u
					//!< Spread weighting over into this many bins on each side
				)
			{
				double const binDelta{ theRing.angleDelta() };
				static prb::Gauss1D const gauss(0., binDelta);

				// add largest weight into main bin
				std::size_t const ndxCurr{ theRing.indexFor(angle) };
				double const offset{ angle - theRing.angleAt(ndxCurr) };
				theBinSums[ndxCurr] += weight * gauss(offset);

				// add decreasing weights into (circularly) adjacent bins
				for (std::size_t dn{0u} ; dn < halfBinSpread ; ++dn)
				{
					double const angDelta{ binDelta * (double)dn };

					int const dnPos{ (int)dn };
					std::size_t const ndxPos
						{ theRing.indexRelativeTo(ndxCurr, dnPos) };
					theBinSums[ndxPos] += weight * gauss(offset + angDelta);

					int const dnNeg{ -dnPos };
					std::size_t const ndxNeg
						{ theRing.indexRelativeTo(ndxCurr, dnNeg) };
					theBinSums[ndxNeg] += weight * gauss(offset - angDelta);
				}
			}

			//! Local peaks in angle data buffer
			inline
			std::vector<std::size_t>
			indicesOfPeaks
				() const
			{
				std::vector<std::size_t> peakNdxs;
				peakNdxs.reserve(size());
				std::size_t const ndxEnd{ size() };
				for (std::size_t ndxCurr{0u} ; ndxCurr < ndxEnd ; ++ndxCurr)
				{
					std::size_t const ndxPrev
						{ theRing.indexRelativeTo(ndxCurr, -1) };
					std::size_t const ndxNext
						{ theRing.indexRelativeTo(ndxCurr,  1) };
					double const & valPrev = theBinSums[ndxPrev];
					double const & valCurr = theBinSums[ndxCurr];
					double const & valNext = theBinSums[ndxNext];
					bool const maybePeak
						{  (! (valCurr < valPrev))
						&& (! (valCurr < valNext))
						};
					if (maybePeak)
					{
						peakNdxs.emplace_back(valCurr);
					}
				}
				return peakNdxs;
			}

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
				oss
					<< "theRing: " << theRing
					<< ' '
					<< "binSize: " << size()
					<< '\n';
				return oss.str();
			}

			//! Description of the buffer contents
			inline
			std::string
			infoStringContents
				( std::string const & title = {}
				) const
			{
				std::ostringstream oss;
				oss << infoString(title) << '\n';
				for (std::size_t nbin{0u} ; nbin < theBinSums.size() ; ++nbin)
				{
					using engabra::g3::io::fixed;
					std::cout
						<< std::setw(3u) << nbin
						<< ' ' << fixed(theRing.angleAt(nbin), 6u, 6u)
						<< ' ' << fixed(theBinSums[nbin], 6u, 6u)
						<< '\n';
				}
				return oss.str();
			}

		}; // AngleProbs


		// create angle value accumulation (circular-wrapping) buffer
		std::size_t const numAngBins{ 32u };
		AngleProbs angleProbs(numAngBins);
		for (pix::EdgePair const & edgePair : edgePairs)
		{
			pix::Grad const meanDir{ edgePair.meanDir(edgeInfos) };
			double const angle{ edgePair.angle(meanDir) };
			double const weight{ edgePair.weightRadialPair() };
			angleProbs.add(angle, weight, 2);
		}

		std::vector<std::size_t> const peakNdxs{ angleProbs.indicesOfPeaks() };


std::cout << angleProbs.infoStringContents("angleProbs") << '\n';

std::cout << '\n';
std::cout << "numWorstCase: " << numWorstCase << '\n';
std::cout << "     numUsed: " << edgePairs.size() << '\n';
std::ofstream ofs("./lineSeg.dat");

		for (pix::EdgePair const & edgePair : edgePairs)
		{
			pix::Grad const meanDir{ edgePair.meanDir(edgeInfos) };
			using engabra::g3::io::fixed;
			ofs
				<< ' ' << meanDir                            // 1,2
				<< ' ' << edgePair.angle(meanDir)            // 3
				<< ' ' << fixed(edgePair.weightFacing())     // 4
				<< ' ' << fixed(edgePair.weightLineGap())    // 5
				<< ' ' << fixed(edgePair.weightRadialPair()) // 6
				<< '\n';
		}


		// draw strongest pixels into grid (for development feedback)
		dat::Grid<float> const magGrid
			(edgeMagGridFor(pixGrid.hwSize(), pixEdgels, numToUse));

		std::string const pixName{ std::format("./sample{:02d}pix.pgm", nn) };
		std::string const magName{ std::format("./sample{:02d}mag.pgm", nn) };
		io::writeStretchPGM(pixName, pixGrid);
		io::writeStretchPGM(magName, magGrid);

		std::cout << '\n';
		std::cout << "pixEdgels.size: " << pixEdgels.size() << '\n';
		std::cout << "Pix data written to '" << pixName << "'\n";
		std::cout << "Mag data written to '" << magName << "'\n";

break;

	}
	std::cout << '\n';

	return 0;
}


