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
 * \brief Declarations for quadloco::img::Area
 *
 */


#include "sigSpan.hpp"
#include "imgSpot.hpp"

#include <array>
#include <sstream>
#include <string>


namespace quadloco
{

namespace dat
{

	//! A 2D region defined by two orthogonal 1D spans
	struct Area
	{
		//! Two independent spans that define the 2D area
		std::array<Span, 2u> theSpans;

		//! For convenience to denote 2D data composed of fraction values
		typedef std::array<double, 2u> Dyad;

		//! Modulo (fractional) portion of someFrac: (0 <= principalFrac < 1)
		inline
		static
		double
		principalFraction
			( double const & someFrac
			)
		{
			double wholFrac{};
			double const partFrac{ std::modf(someFrac, &wholFrac) };
			// modf() rounds toward zero - need "always round downward"
			double offset{ 0. };
			if (someFrac < 0.)
			{
				if (0. != partFrac)
				{
					offset = 1.;
				}
			}
			return partFrac + offset;
		}

		//! Fractional Dyad with components satisfy: 0 <= comp < 1.
		inline
		static
		Dyad
		principalFractionDyad
			( Dyad const & someFracDyad
			)
		{
			return Dyad
				{ principalFraction(someFracDyad[0])
				, principalFraction(someFracDyad[1])
				};
		}


		//! True if both coordinates are not null.
		inline
		bool
		isValid
			() const
		{
			return
				(  theSpans[0].isValid()
				&& theSpans[1].isValid()
				);
		}

		//! True if spot is in area based on simultaneous Span::contains()
		inline
		bool
		contains
			( Spot const & spot
			) const
		{
			return
				(  theSpans[0].contains(spot[0])
				&& theSpans[1].contains(spot[1])
				);
		}

		//! Dyad (std::array<double,2u>) of fractions in each direction at spot
		inline
		Dyad
		fractionDyadAtSpot
			( img::Spot const & spot
			) const
		{
			double const frac0{ theSpans[0].fractionAtValue(spot[0]) };
			double const frac1{ theSpans[1].fractionAtValue(spot[1]) };
			return std::array<double, 2u>{ frac0, frac1 };
		}

		//! Coordinate location at fractional values into the area.
		inline
		img::Spot
		spotAtFractionDyad
			( Dyad const fracDyad
			) const
		{
			double const value0{ theSpans[0].valueAtFraction(fracDyad[0]) };
			double const value1{ theSpans[1].valueAtFraction(fracDyad[1]) };
			return img::Spot{ value0, value1 };
		}

		//! Descriptive information about this instance
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
				<< "span0: " << theSpans[0]
				<< ' ' 
				<< "span1: " << theSpans[1]
				;
			return oss.str();
		}

	}; // Area


} // [dat]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::img::Area const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if span is not null
	inline
	bool
	isValid
		( quadloco::img::Area const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

