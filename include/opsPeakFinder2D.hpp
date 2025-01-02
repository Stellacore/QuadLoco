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
 * \brief Declarations for quadloco::ops::PeakFinder2D namespace
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
	struct PeakFinder2D
	{
		/*! \brief All local peaks (at least one cell inside grid border)
		 *
		 * Peaks are based on neighbor value checking. If the value
		 * at the center of an 8-hood is not less than all 8 neighbors
		 * then the center is considered a peak.
		 *
		 * This is a brute force niave algorithm. It visits every cell
		 * (other than the first and last row or first and last column).
		 * At each evaluation site, it checks all 8 neighbor values.
		 */
		template <typename Type>
		inline
		static
		std::vector<ras::PeakRCV>
		peakRCVs
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

					if (minValue < MM)
					{
						// all (9) pixels in hood are valid
						if (  pix::isValid(TL)
						   && pix::isValid(TM)
						   && pix::isValid(TR)
						   && pix::isValid(ML)
						   && pix::isValid(MM)
						   && pix::isValid(MR)
						   && pix::isValid(BL)
						   && pix::isValid(BM)
						   && pix::isValid(BR)
						   )
						{
							// middle pixel is not less than (8) neighbors
							bool const isPeak
								{  (! (MM < TL))
								&& (! (MM < TM))
								&& (! (MM < TR))
								&& (! (MM < ML))
								&& (! (MM < MR))
								&& (! (MM < BL))
								&& (! (MM < BM))
								&& (! (MM < BR))
								};
							if (isPeak)
							{
								ras::PeakRCV const peakRCV
									{ ras::RowCol{ currRow, currCol }
									, static_cast<double>(MM)
									};
								peakRCVs.emplace_back(peakRCV);

							} // isPeak

						} // all valid

					} // over minValue

				} // currCol

			} // currRow

			return peakRCVs;
		}

		//! Same as peakRCVs followed by sort() to put largest value peak first
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

	}; // PeakFinder2D


} // [ops]

} // [quadloco]

