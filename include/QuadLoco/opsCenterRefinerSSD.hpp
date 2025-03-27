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
#include "QuadLoco/rasChipSpec.hpp"
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
		gridOfAveSSD
			( ras::RowCol const & rcHoodCenterInSrc
			, std::size_t const & fullHood // i.e. (2u*halfHood+1u)
			, prb::Stats<double> * const & ptSrcStats = { nullptr }
			) const
		{
			// allocate and initialize sum-sqr-diff return grid
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
					if (ptSrcStats)
					{
						ptSrcStats->consider((double)fwdSrcVal);
						ptSrcStats->consider((double)revSrcVal);
					}
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
		 * represent an "*average* of SSD values over valid cells"
		 * throughout some neighborhood of interest.
		 *
		 * The ssd (per valid pixel) values have a theoretical minimum
		 * of zero (every source image pixel values is exactly the same
		 * as the diametrically opposite pixel).
		 *
		 * The ssd maximum value depends on the range of actual source
		 * input values (e.g. for full range 8bit image, this could be
		 * 255, for a normalized image, it could be 1.0).  Thus, the
		 * consuming code must provide an indication of the variance
		 * present in the original source grid values.
		 *
		 * Pseudo probabilities are assigned based on the relationship
		 * of the ssdGrid value and the provide variance via the
		 * Gaussian-like pseudo probability value:
		 * \arg std::exp(-(ssdValue / varSrcPix)) };
		 */
		inline
		static
		img::Hit
		hitAtMinimumOf
			( ras::Grid<double> const & ssdGrid
				//!< Per pixel SSD values over neighborhood
			, double const & varSrcPix
				//!< Radiometric standard deviation of source grid values
			)
		{
			img::Hit minHit{};
			if (ssdGrid.isValid() && engabra::g3::isValid(varSrcPix))
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

						double const argSq{ ssdValue / varSrcPix };
						double const prob{ std::exp(-argSq) };

						sumVec = sumVec + prob * locSpot;
						sumProb += prob;

						// Cache probability values to compute variance below
						probGrid(inRC) = prob;
					}
				}

				// Estimate variance (deviation) of best fit location
				if (0. < sumProb)
				{
					img::Spot const aveVec{ (1./sumProb) * sumVec  };
					img::Spot const & minSpot = aveVec;

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
				// gather source image value stats (for use in
				// assessing significance of SSD values)
				// while computing SSD in neighborhood
				prb::Stats<double> srcStats{};
				std::size_t const fullHood{ 2u*theHalfHood + 1u };
				ras::Grid<double> const aveGridSSD
					{ gridOfAveSSD(rcHoodCenterInSrc, fullHood, &srcStats) };

				// upper left corner of ssd evaluation chip
				ras::RowCol const rcChipTL
					{ rcHoodCenterInSrc.row() - theHalfHood
					, rcHoodCenterInSrc.col() - theHalfHood
					};
				ras::ChipSpec const chipSpec{ rcChipTL, aveGridSSD.hwSize() };

				// assume dominant random noise in the (presumably small)
				// neighborhood is due to shot noise (which goes as square 
				// root of intensity. For dark/light patches, the
				// deviation should be sqrt(dark)/sqrt(light) which means
				// the variance should be sum of expected min and expected
				// max values. Assume (ad hoc) that the extreme min and
				// max values are "reasonably" close to the expected min/max.
				double const varBlack{ srcStats.min() };
				double const varWhite{ srcStats.max() };
				double const varSrcPix{ (1./10.) * (varBlack + varWhite) };

				// estimate sub-cell location of minimum
				img::Hit const minHitInChip
					{ hitAtMinimumOf(aveGridSSD, varSrcPix) };
				img::Spot const & minSpotInChip = minHitInChip.location();

				// get full source image location for ssd Chip minimum
				img::Spot const minSpotInGrid
					{ chipSpec.fullSpotForChipSpot(minSpotInChip) };

				double const & prob = minHitInChip.value();
				double const & sigma = minHitInChip.sigma();
				fitHit = img::Hit(minSpotInGrid, prob, sigma);
			}

			return fitHit;
		}

	}; // CenterRefinerSSD


} // [ops]

} // [quadloco]

