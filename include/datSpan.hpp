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
 * \brief Declarations for dat::Span
 *
 */


#include <Engabra>


namespace quadloco
{

namespace dat
{

	//! A half open interval
	struct Span
	{
		double const theBeg{ engabra::g3::null<double>() };
		double const theEnd{ engabra::g3::null<double>() };

		//! True if this instance contains valid data
		inline
		bool
		isValid
			() const
		{
			return
				(  engabra::g3::isValid(theBeg)
				&& engabra::g3::isValid(theEnd)
				&& (theBeg < theEnd)
				);
		}

		//! Length of span
		inline
		double
		magnitude
			() const
		{
			return (theEnd - theBeg);
		}

		//! Is value in half open range: [minInclude <= value < maxExclude]?
		inline
		bool
		contains
			( double const & value
			) const
		{
			bool const inside
				{  (! (value < theBeg))  // beg <= value
				&&    (value < theEnd)   // value < end
				};
			return inside;
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
			using engabra::g3::io::fixed;
			oss << fixed(theBeg) << ' ' << fixed(theEnd) ;
			return oss.str();
		}

	};


} // [dat]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::dat::Span const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::dat::Span const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]


