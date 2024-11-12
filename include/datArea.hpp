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
 * \brief Declarations for quadloco::dat::Area
 *
 */


#include "datSpan.hpp"
#include "datSpot.hpp"

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
		std::array<Span, 2u> theSpans;

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
		, quadloco::dat::Area const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if span is not null
	inline
	bool
	isValid
		( quadloco::dat::Area const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

