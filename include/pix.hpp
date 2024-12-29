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
 * \brief Top level file for quadloco::pix namespace
 *
 */


#include "imgSpan.hpp"
#include "rasGrid.hpp"

#include <cmath>
#include <cstdint>
#include <limits>


namespace quadloco
{

/*! \brief Namespaced functions and utilities for pixel (ras::Grid) elements.
 */
namespace pix
{

	//
	// Floating pixel type values
	//

	//! Floating point image pixel type
	using fpix_t = float;

	//! Null value for missing or not available pixel data (NaN)
	constexpr fpix_t fNull{ std::numeric_limits<fpix_t>::quiet_NaN() };

	//! Underexposed pixel (negative infinity)
	constexpr fpix_t fUndr{ -std::numeric_limits<fpix_t>::infinity() };

	//! darkest informative pixel (lowest)
	constexpr fpix_t fDark{  std::numeric_limits<fpix_t>::lowest() };

	//! brightest informative pixel (max)
	constexpr fpix_t fLite{  std::numeric_limits<fpix_t>::max() };

	//! Overexposed pixel (positive infinity)
	constexpr fpix_t fOver{  std::numeric_limits<fpix_t>::infinity() };


	//
	// Integer pixel type values
	//

	//! Null value for missing or not available pixel data (NaN)
	constexpr uint8_t u8Null{ 0u };

	//! Underexposed pixel (negative infinity)
	constexpr uint8_t u8Undr{ 1u };

	//! darkest informative pixel (lowest)
	constexpr uint8_t u8Dark{ 2u };

	//! brightest informative pixel (max)
	constexpr uint8_t u8Lite{ 254u };

	//! Overexposed pixel (positive infinity)
	constexpr uint8_t u8Over{ 255u };


	//
	// Null values
	//

	//! A "not available" indicator for data of Type
	template <typename Type>
	inline
	constexpr
	Type
	null
		()
	{
		return std::numeric_limits<Type>::quiet_NaN();
	}



	//
	// Functions
	//


	//! True if pixel is not null
	inline
	bool
	isValid
		( fpix_t const & fpix
		)
	{
		return (std::isnormal(fpix) || (0. == fpix));
	}

	//! Min/Max values from a collection with only isValid() inputs considered
	template <typename FwdIter, typename PixType = float>
	inline
	std::pair<PixType, PixType>
	validMinMax
		( FwdIter const & beg
		, FwdIter const & end
		)
	{
		using namespace quadloco;
		std::pair<PixType, PixType> minmax
			{ std::numeric_limits<PixType>::quiet_NaN()
			, std::numeric_limits<PixType>::quiet_NaN()
			};
		if (beg != end)
		{
			PixType & min = minmax.first;
			PixType & max = minmax.second;
			for (FwdIter iter{beg} ; end != iter ; ++iter)
			{
				PixType const & pixVal = *iter;
				if (isValid(pixVal))
				{
					if ((! isValid(min)) || (pixVal < min))
					{
						min = pixVal;
					}
					if ((! isValid(max)) || (max < pixVal))
					{
						max = pixVal;
					}
				}
			}
		}
		return minmax;
	}

	//! Span with begin/end at smallest/largest (valid) values in fGrid
	template <typename PixType>
	inline
	img::Span
	fullSpanFor
		( ras::Grid<PixType> const & fGrid
		)
	{
		std::pair<PixType, PixType> const fMinMax
			{ validMinMax(fGrid.cbegin(), fGrid.cend()) };
		PixType const & fMin = fMinMax.first;
		// bump max by a tiny amount so that largest value *is* included
		PixType const & fMax = fMinMax.second;
		PixType const delta{ fMax - fMin };
		constexpr PixType eps{ std::numeric_limits<PixType>::epsilon() };
		PixType const useMax{ fMax * (1.f + delta * eps) };
		return quadloco::img::Span{ (double)fMin, (double)useMax };
	}

	/*! uint8_t value corresponding to fpix value within fSpan range
	 *
	 * Returned upix values are mapped as
	 * \arg u8Undr : fpix  < fSpan.min()
	 * \arg u8Dark : fpix == fSpan.min()
	 * \arg   ...  : fSpan.min() <= fpix < fSpan.max()
	 * \arg u8Lite : fpix == (fSpan.max() - epsilon) 
	 * \arg u8Over : fSpan.max() <= fpix
	 *
	 * An fgrid value at fSpan.begin() maps to u8Dark
	 */
	inline
	uint8_t
	uPix8
		( fpix_t const & fpix
		, img::Span const & fSpan
		)
	{
		uint8_t upix{ u8Null };
		if (isValid(fpix))
		{
			fpix_t const & fBeg = fSpan.min();
			fpix_t const & fEnd = fSpan.max();
			if (fpix < fBeg)
			{
				upix = u8Undr;
			}
			else
			if (fpix < fEnd)
			{
				// "Normal" range between Dark and Lite
				// NOTE: uSpan.max() is *out* of valid span so that
				//       fraction interp goes only to (u8Over-1)==u8Lite
				constexpr img::Span uSpan{ (double)u8Dark, (double)u8Over };
				double const frac{ fSpan.fractionAtValue((double)fpix) };
				double const value{ uSpan.valueAtFraction(frac) };
				upix = static_cast<uint8_t>(std::floor(value));
			}
			else
			{
				upix = u8Over;
			}
		}
		return upix;
	}

	/*! \brief A uint8_t grid that maps fgrid values via fSpan range.
	 *
	 * ref uPix8() for details on value mapping.
	 */
	inline
	ras::Grid<uint8_t>
	uGrid8
		( ras::Grid<fpix_t> const & fgrid
		, img::Span const & fSpan
		)
	{
		ras::Grid<uint8_t> ugrid{ fgrid.hwSize() };

		ras::Grid<fpix_t>::const_iterator itIn{ fgrid.cbegin() };
		ras::Grid<uint8_t>::iterator itOut{ ugrid.begin() };
		while (ugrid.end() != itOut)
		{
			*itOut++ = uPix8(*itIn++, fSpan);
		}
		return ugrid;
	}


} // [pix]

} // [quadloco]

