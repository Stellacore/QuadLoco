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


#pragma once


/*! \file
 * \brief Declarations for quadloco::ops::CenterRefinerSSD namespace
 *
 */


#include "QuadLoco/cast.hpp"
#include "QuadLoco/imgHit.hpp"
#include "QuadLoco/imgSpot.hpp"
#include "QuadLoco/prbStats.hpp"
#include "QuadLoco/rasGrid.hpp"
#include "QuadLoco/rasRelRC.hpp"
#include "QuadLoco/rasRowCol.hpp"
#include "QuadLoco/valSpan.hpp"

#include <Engabra>


namespace quadloco
{

namespace ops
{

	//! \brief Center finding functions assocaited with an external source grid
	class CenterRefinerSSD
	{
		//! Access into external source grid
		ras::Grid<float> const * const thePtSrcGrid{ nullptr };

		//! Neighborhood half-size around candidate center location
		std::size_t const theHalfHood{};

		//! Filter half-size for rotation symmetry evaluation
		std::size_t const theHalfCorr{};

		//! Relative (signed) indices relative to candidate center peak
		std::vector<ras::RelRC> theHoodRelRCs{};

		//! Relative (signed) indices defining rotation symmetry filter inputs
		std::vector<ras::RelRC> theCorrRelRCs{};

		//! Relative row,col offsets for defining evaluation box
		inline
		static
		std::vector<ras::RelRC>
		boxRelRCs
			( std::size_t const & halfHood
			)
		{
			std::vector<ras::RelRC> relRCs{};

			// compute the relative row,col offsets for neighborhood box
			std::size_t const fullHood{ 2u*halfHood + 1u };

			// compute evaluation neighborhood relative offsets
			relRCs.reserve(fullHood * fullHood);
			for (std::size_t rowHood{0u} ; rowHood < fullHood ; ++rowHood)
			{
				int const rowRel{ (int)rowHood - (int)halfHood };
				for (std::size_t colHood{0u} ; colHood < fullHood ; ++colHood)
				{
					int const colRel{ (int)colHood - (int)halfHood };
					ras::RelRC const relRC{ rowRel, colRel };
					relRCs.emplace_back(relRC);
				}
			}

			return relRCs;
		}

	public:

		//! Attach refiner to source grid with specific refinement parameters.
		inline
		explicit
		CenterRefinerSSD
			( ras::Grid<float> const * const ptSrcGrid
				//!< Access to source grid (under consumer management)
			, std::size_t const & halfHood = 2u
				//!< Half size of (2u*halfHood+1) neighborhood to search
			, std::size_t const & halfCorr = 5u
				//!< Radius of rotation filter to use over all box cells
			)
			: thePtSrcGrid{ ptSrcGrid }
			, theHalfHood{ halfHood }
			, theHalfCorr{ halfCorr }
			, theHoodRelRCs{ boxRelRCs(theHalfHood) }
			, theCorrRelRCs{ boxRelRCs(theHalfCorr) }
		{ }

		/*! \brief Grid of sum-squared-differences centered on source location
		 *
		 * The return value is the *expected* squared difference per *valid*
		 * pair of pixels in the neighboor hood. E.g. it is the sum of 
		 * squared difference of valid pixels divided by the number of
		 * valid (diametrically opposite) pixels in the neighborhood.
		 */
		inline
		ras::Grid<double>
		gridOfHoodPerPixSSD
			( ras::RowCol const & rcHoodCenterInSrc
			) const
		{
			// allocate and initialize sum-sqr-diff return grid
			std::size_t const fullHood{ 2u*theHalfHood + 1u };
			ras::Grid<double> ssdGrid(fullHood, fullHood);
			std::fill(ssdGrid.begin(), ssdGrid.end(), pix::null<float>());

			// useful shorthand
			ras::Grid<float> const & srcGrid = *thePtSrcGrid;
			using CorrIter = ras::Grid<double>::iterator;
			using FwdIter = std::vector<ras::RelRC>::const_iterator;
			using RevIter = std::vector<ras::RelRC>::const_reverse_iterator;

			// loop over evaluation neighborhood (corresponding to output grid)
			CorrIter outCorrIter{ ssdGrid.begin() };
			double count{ 0. };
			for (ras::RelRC const & hoodRelRC : theHoodRelRCs)
			{
				ras::RowCol const rcHood0
					{ hoodRelRC.srcRowCol(rcHoodCenterInSrc) };

				// Evaluate rotation symmetry at each point in evaluation
				// neighborhood.  Use two iterators running from opposite
				// directions to provide half-turn filter geometry
				std::size_t const halfCorr{ theCorrRelRCs.size() / 2u };
				FwdIter const fwdHalf{ theCorrRelRCs.cbegin() + halfCorr };
				FwdIter fwdIter{theCorrRelRCs.cbegin()};
				RevIter revIter{theCorrRelRCs.crbegin()};

				// compute filter response at this evaluation location
				double sumSqDif{ 0. };
				for ( ; fwdHalf != fwdIter ; ++fwdIter, ++revIter)
				{
					ras::RowCol const fwdRowCol
						{ fwdIter->srcRowCol(rcHood0) };
					ras::RowCol const revRowCol
						{ revIter->srcRowCol(rcHood0) };

					float const & fwdSrcVal = srcGrid(fwdRowCol);
					float const & revSrcVal = srcGrid(revRowCol);
					if (pix::isValid(fwdSrcVal) && pix::isValid(revSrcVal))
					{
						double const diff
							{ static_cast<double>(fwdSrcVal - revSrcVal) };
						sumSqDif += diff*diff;
						count += 1.;
					}
				}

				// compute 'expected ssd' per valid input pixel-pair
				if (0. < count)
				{
					double const aveSqDif{ sumSqDif / count };
					*outCorrIter++ = aveSqDif;
				}

			} // hood locations

			return ssdGrid;
		}

		/*! \brief A sub-cell estimate of location for minimum within ssdGrid
		 *
		 * Input grid, ssdGrid, is assumed to have cell values that
		 * represent "SSD per (valid)pixel" through some neighborhood of
		 * interest.
		 *
		 * The ssd (per valid pixel) values have a theoretical minimum
		 * of zero (every source image pixel values is exactly the same
		 * as the diametrically opposite pixel).
		 *
		 * The ssd maximum value depends on the range of actual source
		 * input values (e.g. for full range 8bit image, could be 255, but
		 * in general max will be some arbitrary value based on image
		 * exposure, scene content, etc).
		 *
		 * For conversion to pseudo probabilities,
		 * \arg Define a "sigma" value
		 * \arg Compute guassian-like function: exp(-sq(ssdValue/sigma));
		 *
		 * The sigma value is *arbitrarily* set to 1/4 of the maximum
		 * ssdValue in the ssdGrid. This results in pseudo probabilities
		 * that are very small for ssdGrid values that are near the
		 * "worst" in the neighborhood.
		 *
		 * The "best" pseudo probability has a value of
		 * \arg bestProb = exp(-sq(ssdGridMin/sigma).
		 * \arg bestProb = exp(-sq(ssdGridMin/(.25*ssdGridMax))).
		 *
		 * If the minSSD is near zero, then the best pseudo prob will
		 * be near 1.0 no matter what the sigma value is (as long as it
		 * is not zero).  For the general case, the pseudo probability
		 * will be a maximum at the ssdGrid minimum, but will have a
		 * value that is rather arbitrarily scaled.
		 */
		inline
		static
		img::Hit
		hitAtMinimumOf
			( ras::Grid<double> const & ssdGrid
			)
		{
			img::Hit minHit{};
			// gather stats over entire input grid
			prb::Stats<double> const ssdStats
				(ssdGrid.cbegin(), ssdGrid.cend());

			// define pseudo-probability deviation
			double sigma{ engabra::g3::null<double>() };
			if (ssdStats.isValid())
			{
				// scale for pseudo probability distribution over ssd values
				// arbitrary such that maximum ssd in neighborhood will
				// have a very small pseudo prob.
				constexpr double sigmaFracOfMax{ .25 };
				sigma = sigmaFracOfMax * ssdStats.max();
			}

			if (engabra::g3::isValid(sigma))
			{
				ras::Grid<double> probGrid(ssdGrid.hwSize());
				std::fill(probGrid.begin(), probGrid.end(), 0.);

				// Estimate best fit location (ssdGrid minimum)
				img::Vector<double> sumVec{ 0., 0. };
				double sumProb{ 0. };
				for (ras::Grid<double>::const_iterator
					iter{ssdGrid.cbegin()} ; ssdGrid.cend() != iter ; ++iter)
				{
					// compute weighted minimum over all elements of input
					// grid (using a pseudo-probability weighting function
					// to favor smaller (better) ssdGrid values).
					double const & ssdValue = *iter;
					if (engabra::g3::isValid(ssdValue))
					{
						ras::RowCol const inRC{ ssdGrid.rasRowColFor(iter) };
						img::Spot const locSpot{ cast::imgSpot(inRC) };

						double const arg{ ssdValue / sigma };
						double const prob{ std::exp(-arg*arg) };

						sumVec = sumVec + prob * locSpot;
						sumProb += prob;

						// Cache probability values to compute variance below
						probGrid(inRC) = prob;
					}
				}

				// Estimate variance (deviation) of best fit location
				if (0. < sumProb)
				{
					// The SSD computations are based on matching areas
					// of grid cells and the weighted average computed
					// here is related to the center of the cells.
					// Therefore, add a half cell to recognize this
					img::Vector<double> const halfCell{ .5, .5 };
					img::Vector<double> const aveVec
						{ (1./sumProb) * sumVec  + halfCell };
					img::Spot const minSpot{ cast::imgSpot(aveVec) };

					// Estimate variance
					// TODO pbly could optimize to loop only near to minSpot
					double sumSq{ 0. };
					double sumProb{ 0. };
					for (ras::Grid<double>::const_iterator
						iter{ssdGrid.cbegin()}
						; ssdGrid.cend() != iter ; ++iter)
					{
						double const & ssdValue = *iter;
						if (engabra::g3::isValid(ssdValue))
						{
							ras::RowCol const inRC
								{ ssdGrid.rasRowColFor(iter) };
							img::Spot const locSpot{ cast::imgSpot(inRC) };
							img::Spot const relSpot{ locSpot - minSpot };
							double const relMag{ magnitude(relSpot) };

							double const & prob = probGrid(inRC);
							sumSq += prob * relMag;
							sumProb += prob;
						}
					}
					double const var{ sumSq / sumProb };
					double const sigma{ std::sqrt(var) };
					double const probAtMin
						{ probGrid(cast::rasRowCol(minSpot)) };
					minHit = img::Hit(minSpot, probAtMin, sigma);
				}
			}
			return minHit;
		}


		/*! \brief Refine nominal center location using box neighborhood.
		 *
		 * At each cell within neighborhood box, run a half-turn rotation
		 * correlation filter. Compute and return a sub-cell location of
		 * the peak response from this filter.
		 */
		inline
		img::Hit
		fitHitNear
			( ras::RowCol const & rcHoodCenterInSrc
				//!< Center of window in which to search
			) const
		{
			img::Hit fitHit{};
			std::size_t maxRad{ theHalfHood + theHalfCorr };

			std::size_t const rowHood0{ rcHoodCenterInSrc.row() };
			std::size_t const colHood0{ rcHoodCenterInSrc.col() };

			// check if neighborhood filter size fits within source grid
			bool isInterior{ false };
			if ( thePtSrcGrid
			  && (2u*maxRad < thePtSrcGrid->high())
			  && (2u*maxRad < thePtSrcGrid->wide())
			   )
			{
				std::size_t rowMax{ thePtSrcGrid->high() - maxRad };
				std::size_t colMax{ thePtSrcGrid->wide() - maxRad };
				if ( (maxRad < rowHood0)
				  && (maxRad < colHood0)
				  && (rowHood0 < rowMax)
				  && (colHood0 < colMax)
				   )
				{
					isInterior = true;
				}
			}

			if (isInterior)
			{
				// compute SSD in neighborhood
				ras::Grid<double> const ssdGrid
					{ gridOfHoodPerPixSSD(rcHoodCenterInSrc) };

				// estimate sub-cell location of minimum
				img::Hit const minHitInGrid
					{ hitAtMinimumOf(ssdGrid) };
				img::Spot const & minSpotInGrid = minHitInGrid.location();

				// adjust ssdGrid location into full source image
				img::Spot const fitSpot
					{ minSpotInGrid.row()
						+ (double)(rcHoodCenterInSrc.row())
						- (double)(theHalfHood)
					, minSpotInGrid.col()
						+ (double)(rcHoodCenterInSrc.col())
						- (double)(theHalfHood)
					};

				double const & prob = minHitInGrid.value();
				double const & sigma = minHitInGrid.sigma();
				fitHit = img::Hit(fitSpot, prob, sigma);
			}

			return fitHit;
		}

	}; // CenterRefinerSSD


} // [ops]

} // [quadloco]

