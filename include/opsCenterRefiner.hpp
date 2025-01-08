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
 * \brief Declarations for quadloco::ops::CenterRefiner namespace
 *
 */


#include "cast.hpp"
#include "imgSpan.hpp"
#include "imgSpot.hpp"
#include "prbStats.hpp"
#include "rasGrid.hpp"
#include "rasRelRC.hpp"
#include "rasRowCol.hpp"

#include <Engabra>


namespace quadloco
{

namespace ops
{

	//! \brief Center finding functions assocaited with an external source grid
	class CenterRefiner
	{
		// access into external source grid
		ras::Grid<float> const * const thePtSrcGrid{ nullptr };

		std::size_t const theHalfHood{};
		std::size_t const theHalfCorr{};

		std::vector<ras::RelRC> theHoodRelRCs{};
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

		inline
		explicit
		CenterRefiner
			( ras::Grid<float> const * const ptSrcGrid
				//!< Access to source grid (under consumer management)
			, std::size_t const & halfHood = 2u
				//!< Half size of (2u*halfHood+1) neighborhood to search
			, std::size_t const & halfCorr = 5u
				//!< Radius of filter to use over all box cells
			)
			: thePtSrcGrid{ ptSrcGrid }
			, theHalfHood{ halfHood }
			, theHalfCorr{ halfCorr }
			, theHoodRelRCs{ boxRelRCs(theHalfHood) }
			, theCorrRelRCs{ boxRelRCs(theHalfCorr) }
		{ }

		//! Grid of sum-squared-differences centered on source location
		inline
		ras::Grid<double>
		ssdHoodGrid
			( ras::RowCol const & rcHoodCenterInSrc
			) const
		{
			// allocate and initialize sum-sqr-diff return grid
			std::size_t const fullHood{ 2u*theHalfHood + 1u };
			ras::Grid<double> ssdGrid(fullHood, fullHood);
			std::fill(ssdGrid.begin(), ssdGrid.end(), 0.);

			// useful shorthand
			ras::Grid<float> const & srcGrid = *thePtSrcGrid;
			using CorrIter = ras::Grid<double>::iterator;
			using FwdIter = std::vector<ras::RelRC>::const_iterator;
			using RevIter = std::vector<ras::RelRC>::const_reverse_iterator;

			// loop over evaluation neighborhood (corresponding to output grid)
			CorrIter outCorrIter{ ssdGrid.begin() };
			for (ras::RelRC const & hoodRelRC : theHoodRelRCs)
			{
				ras::RowCol const rcHood0
					{ hoodRelRC.srcRowCol(rcHoodCenterInSrc) };

				// Use two iterators running from opposite directions
				// to provide half-turn filter geometry
				std::size_t const halfCorr{ theCorrRelRCs.size() / 2u };
				FwdIter const fwdHalf{ theCorrRelRCs.cbegin() + halfCorr };
				FwdIter fwdIter{theCorrRelRCs.cbegin()};
				RevIter revIter{theCorrRelRCs.crbegin()};

				// computing SSD over half-turn symmetric filter window
				double sum{ 0. };
				for ( ; fwdHalf != fwdIter ; ++fwdIter, ++revIter)
				{
					ras::RowCol const fwdRowCol
						{ fwdIter->srcRowCol(rcHood0) };
					ras::RowCol const revRowCol
						{ revIter->srcRowCol(rcHood0) };

					float const & fwdSrcVal = srcGrid(fwdRowCol);
					float const & revSrcVal = srcGrid(revRowCol);
					double const diff
						{ static_cast<double>(fwdSrcVal - revSrcVal) };
					sum += diff*diff;
				}
				*outCorrIter++ = sum;

			} // hood locations

			return ssdGrid;
		}

		//! A sub-cell estimate of location for minimum
		inline
		static
		img::Spot
		spotAtMinSSD
			( ras::Grid<double> const & ssdGrid
			)
		{
			img::Spot minSpot{};
			prb::Stats<double> const ssdStats(ssdGrid.cbegin(), ssdGrid.cend());
			img::Span const ssdSpan{ ssdStats.min(), ssdStats.max() };
			if (ssdSpan.isValid())
			{
				img::Vector<double> sumVec{ 0., 0. };
				double sumProb{ 0. };
				for (ras::Grid<double>::const_iterator
					iter{ssdGrid.cbegin()} ; ssdGrid.cend() != iter ; ++iter)
				{
					double const & ssdValue = *iter;
					if (engabra::g3::isValid(ssdValue))
					{
						double const frac{ ssdSpan.fractionAtValue(ssdValue) };
						double const arg{ 4. * frac };
						double const prob{ std::exp(-arg*arg) };
						img::Spot const locSpot
							{ cast::imgSpot(ssdGrid.rasRowColFor(iter)) };
						sumVec = sumVec + prob * locSpot;
						sumProb += prob;
					}
				}
				if (0. < sumProb)
				{
					// The SSD computations are based on matching areas
					// of grid cells and the weighted average computed
					// here is related to the cnter of the cells.
					// Therefore, add a half cell to recognize this
					img::Vector<double> const halfCell{ .5, .5 };
					img::Vector<double> const aveVec
						{ (1./sumProb) * sumVec  + halfCell };
					minSpot = cast::imgSpot(aveVec);
				}
			}
			return minSpot;
		}


		/*! \brief Refine nominal center location using box neighborhood.
		 *
		 * At each cell within neighborhood box, run a half-turn rotation
		 * correlation filter. Compute and return a sub-cell location of
		 * the peak response from this filter.
		 */
		inline
		img::Spot
		fitSpotNear
			( ras::RowCol const & rcHoodCenterInSrc
				//!< Center of window in which to search
			) const
		{
			img::Spot fitSpot{};
			std::size_t maxRad{ theHalfHood + theHalfCorr };

			std::size_t const rowHood0{ rcHoodCenterInSrc.row() };
			std::size_t const colHood0{ rcHoodCenterInSrc.col() };

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
					{ ssdHoodGrid(rcHoodCenterInSrc) };

				// estimate sub-cell location of minimum
				img::Spot const minSpotInGrid{ spotAtMinSSD(ssdGrid) };

				// adjust ssdGrid location into full source image
				fitSpot = img::Spot
					{ minSpotInGrid.row()
						+ (double)(rcHoodCenterInSrc.row())
						- (double)(theHalfHood)
					, minSpotInGrid.col()
						+ (double)(rcHoodCenterInSrc.col())
						- (double)(theHalfHood)
					};
			}

			return fitSpot;
		}

	}; // CenterRefiner


} // [ops]

} // [quadloco]

