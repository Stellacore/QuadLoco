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
 * \brief Declarations for quadloco::dat::SizeMap
 *
 */


#include "datSizeHW.hpp"
#include "datSpan.hpp"
#include "datSpot.hpp"

#include <array>


namespace quadloco
{

namespace dat
{
	struct SizeMap
	{
		std::array<Span, 2u> const theSpanGrids{};
		std::array<Span, 2u> const theSpanAreas{};

		//! A mapping between "grid" space and "area" space
		inline
		explicit
		SizeMap
			( SizeHW const & hwGridSize
			, Span const & areaSpan0
			, Span const & areaSpan1
			)
			: theSpanGrids
				{ Span{ 0., (double)hwGridSize.high() }
				, Span{ 0., (double)hwGridSize.wide() }
				}
			, theSpanAreas
				{ areaSpan0
				, areaSpan1
				}
		{ }

		//! Area spot associated with grid spot location
		inline
		Spot
		areaSpotForGridSpot
			( Spot const & gridSpot
			) const
		{
			double const frac0
				{ theSpanGrids[0].fractionAtValue(gridSpot[0]) };
			double const frac1
				{ theSpanGrids[1].fractionAtValue(gridSpot[1]) };

			double const areaValue0
				{ theSpanAreas[0].valueAtFraction(frac0) };
			double const areaValue1
				{ theSpanAreas[1].valueAtFraction(frac1) };

			return Spot{ areaValue0, areaValue1 };
		}

		//! Grid spot location associated with area spot
		inline
		Spot
		gridSpotForAreaSpot
			( Spot const & areaSpot
			) const
		{
			double const frac0
				{ theSpanAreas[0].fractionAtValue(areaSpot[0]) };
			double const frac1
				{ theSpanAreas[1].fractionAtValue(areaSpot[1]) };

			double const gridValue0
				{ theSpanGrids[0].valueAtFraction(frac0) };
			double const gridValue1
				{ theSpanGrids[1].valueAtFraction(frac1) };

			return Spot{ gridValue0, gridValue1 };
		}

	}; // SizeMap


} // [dat]

} // [quadloco]

