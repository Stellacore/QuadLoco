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
 * \brief Declarations for quadloco::ops::AllPeaks2D namespace
 *
 */


#include "pix.hpp"
#include "rasGrid.hpp"
#include "rasPeakRCV.hpp"
#include "rasRowCol.hpp"

#include <Engabra>

#include <limits>
#include <vector>


namespace quadloco
{

namespace ops
{
	//! \brief Group of functions for operating on 2D peaks
	class AllPeaks2D
	{
		std::vector<ras::PeakRCV> thePeakRCVs{};

	public:

		/*! \brief All local peaks (at least one cell inside grid border)
		 *
		 * Peaks are based on neighbor value checking. If the value
		 * at the center of an 8-hood is not less than all 8 neighbors
		 * then the center is considered a peak.
		 *
		 * This is a brute force niave algorithm. It visits every cell
		 * (other than the first and last row or first and last column).
		 * At each evaluation site, it checks all 8 neighbor values.
		 *
		 * \note The PeakRCV instances are returned in *ORDER ENCOUNTERED*.
		 * The resulting array can be sorted to put largest peak at 
		 * (one or other) end of collection.  Alternatively the method
		 * sortedPeakRCVs() can be used to return top peaks in sorted order.
		 */
		template <typename Type>
		inline
		static
		std::vector<ras::PeakRCV>
		unsortedPeakRCVs
			( ras::Grid<Type> const & fGrid
			, Type const & minValue = std::numeric_limits<Type>::epsilon()
			)
		{
			std::vector<ras::PeakRCV> peakRCVs;
			peakRCVs.reserve(4u*1024u);

			std::size_t const high{ fGrid.high() };
			std::size_t const wide{ fGrid.wide() };
			std::size_t const lastRow{ high - 1u };
			std::size_t const lastCol{ wide - 1u };
			for (std::size_t currRow{1u} ; currRow < lastRow ; ++currRow)
			{
				std::size_t const prevRow{ currRow - 1u };
				std::size_t const nextRow{ currRow + 1u };
				for (std::size_t currCol{1u} ; currCol < lastCol ; ++currCol)
				{
					std::size_t const prevCol{ currCol - 1u };
					std::size_t const nextCol{ currCol + 1u };

					Type const & TL = fGrid(prevRow, prevCol);
					Type const & TM = fGrid(prevRow, currCol);
					Type const & TR = fGrid(prevRow, nextCol);

					Type const & ML = fGrid(currRow, prevCol);
					Type const & MM = fGrid(currRow, currCol);
					Type const & MR = fGrid(currRow, nextCol);

					Type const & BL = fGrid(nextRow, prevCol);
					Type const & BM = fGrid(nextRow, currCol);
					Type const & BR = fGrid(nextRow, nextCol);

					if (pix::isValid(MM) && (minValue < MM))
					{
						// treat null surrounding peaks as less than valid MM
						bool const isPeak
							{  ( (! pix::isValid(TL)) || (! (MM < TL)) )
							&& ( (! pix::isValid(TM)) || (! (MM < TM)) )
							&& ( (! pix::isValid(TR)) || (! (MM < TR)) )
							//
							&& ( (! pix::isValid(ML)) || (! (MM < ML)) )
							&& ( (! pix::isValid(MR)) || (! (MM < MR)) )
							//
							&& ( (! pix::isValid(BL)) || (! (MM < BL)) )
							&& ( (! pix::isValid(BM)) || (! (MM < BM)) )
							&& ( (! pix::isValid(BR)) || (! (MM < BR)) )
							};
						if (isPeak)
						{
							ras::PeakRCV const peakRCV
								{ ras::RowCol{ currRow, currCol }
								, static_cast<double>(MM)
								};
							peakRCVs.emplace_back(peakRCV);
						}

					} // over minValue

				} // currCol

			} // currRow

			return peakRCVs;
		}

		/*! Same as unsortedPeakRCVs followed by sort() and resize().
		 *
		 * The returned array contains PeakRCV values in sorted order
		 * with the largest peak value first (e.g. in result.front()).
		 */
		template <typename Type>
		inline
		static
		std::vector<ras::PeakRCV>
		sortedPeakRCVs
			( ras::Grid<Type> const & fGrid
			, Type const & minValue = std::numeric_limits<Type>::epsilon()
			)
		{
			std::vector<ras::PeakRCV> peaks{ peakRCVs(fGrid, minValue) };
			sort(peaks.rbegin(), peaks.rend());
			return peaks;
		};

		/*! \brief Measure of how much largest peak stands out from second
		 *
		 * For "first" and "second" largest peak values are extracted
		 * from the *SORTED* collection, peakRCVs. Denote these as
		 * F and S respectively with values:
		 * \arg F = peakRCVs[ndxF]
		 * \arg S = peakRCVs[ndxF+1u]
		 *
		 * The returned value is:
		 *
		 * General Case:
		 * \arg (F-S)/F : A value of 1 indicates a strong peak
		 *                  relative to S. A value of 0 implies F==S.
		 *
		 * Special Cases:
		 * \arg 0.      : if F==S or 0==F
		 * \arg NaN     : no peaks: (ndxF and ndxS=ndxF+1 both past end)
		 * \arg 1.      : only one peak at ndxF (ndxS=ndxF+1 is past end)
		 */
		inline
		static
		double
		distinction
			( std::vector<ras::PeakRCV> const & sortedPeakRCVs
			, std::size_t const & ndxF = 0u
			)
		{
			double dist{ std::numeric_limits<double>::quiet_NaN() };
			if (! sortedPeakRCVs.empty())
			{
				dist = 1.;
				std::size_t const ndxS{ ndxF + 1u };
				if (ndxS < sortedPeakRCVs.size())
				{
					double const & vF = sortedPeakRCVs[ndxF].theValue;
					double const & vS = sortedPeakRCVs[ndxS].theValue;
					if (std::numeric_limits<double>::epsilon() < vF)
					{
						dist = (vF - vS) / vF;
					}
					else
					{
						dist = 0.;
					}
				}
			}
			return dist;
		}

	public:

		//! \brief Search for all 8-hood peaks using peakRCVs() function.
		template <typename Type>
		inline
		explicit
		AllPeaks2D
			( ras::Grid<Type> const & fGrid
			, Type const & minValue = std::numeric_limits<Type>::epsilon()
			)
			: thePeakRCVs{ unsortedPeakRCVs(fGrid, minValue) }
		{ }

		//! \brief Access to all found peak row/col/values.
		inline
		std::vector<ras::PeakRCV> const &
		unsortedPeakRCVs
			() const
		{
			return thePeakRCVs;
		}

		//! Number of peaks found
		inline
		std::size_t
		size
			() const
		{
			return thePeakRCVs.size();
		}

		//! \brief A copy of peakRCVs() sorted with largest value peak first.
		inline
		std::vector<ras::PeakRCV>
		largestPeakRCVs
			( std::size_t const & numToGet
				= std::numeric_limits<std::size_t>::max() 
			) const
		{
			// get unsorted order (e.g. order encountered)
			std::vector<ras::PeakRCV> allPeaks{ unsortedPeakRCVs() };

			if (! (numToGet < allPeaks.size()))
			{
				// sort by peak value - and return all elements
				std::sort(allPeaks.rbegin(), allPeaks.rend());
			}
			else
			{
				// sort by peak value and only return number requested
				std::partial_sort
					( allPeaks.begin()
					, allPeaks.begin() + numToGet
					, allPeaks.end()
					, [] (ras::PeakRCV const & v1, ras::PeakRCV const & v2)
						{ return (v2 < v1); }
					);
				allPeaks.resize(numToGet);
			}
			return allPeaks;
		}

	}; // AllPeaks2D


} // [ops]

} // [quadloco]

