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
 * \brief Declarations for quadloco::pix::Gradel
 *
 */


#include <array>

#include <Engabra>

#include <format>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>


namespace quadloco
{

namespace pix
{

	/*! \brief GRADent ELement structure representing a directed edge gradient.
	 *
	 * \note For computing gradient values over entire dat::Grid instances
	 * refer to functions in pixgrid.hpp (e.g. gradelGridFor()).
	 */
	class Gradel
	{
		//! Internal data representation (could probably use 16-bit floats?)
		std::array<float, 2u> theComps
			{ engabra::g3::null<float>()
			, engabra::g3::null<float>()
			};

	public: // static functions

		//! Functor for formatting Gradel data into a string
		struct Formatter
		{
			//! std::format to apply to each of the two theComps
			std::string const theFormatEach{ "{:4.1f}" };

			inline
			std::string
			operator()
				( Gradel const & elem
				) const
			{
				std::ostringstream fmt;
				fmt << '(' << theFormatEach << ',' << theFormatEach << ')';
				return std::vformat
					( fmt.str()
					, std::make_format_args(elem[0], elem[1])
					);
			}

		}; // Formatter

	public:

		//! Default construction of null instance
		inline
		explicit
		Gradel
			() = default;

		//! Value construction
		inline
		explicit
		Gradel
			( std::array<float, 2u> const & gradComps
			)
			: theComps{ gradComps }
		{ }

		//! Value construction
		inline
		explicit
		Gradel
			( float const & comp0
			, float const & comp1
			)
			: theComps
				{ static_cast<float>(comp0)
				, static_cast<float>(comp1)
				}
		{ }

		//! Read-only access to ndx-the coordinate - NO BOUNDS CHECKING
		inline
		float const &
		operator[]
			( std::size_t const & ndx
			) const
		{
			return theComps[ndx];
		}

		//! True if this instance is not null
		inline
		bool
		isValid
			() const
		{
			return
				(  engabra::g3::isValid(theComps[0])
				&& engabra::g3::isValid(theComps[1])
				);
		}

		//! True if this and other instance data are same within tol
		inline
		bool
		nearlyEquals
			( Gradel const & other
			, double const & tol = std::numeric_limits<float>::epsilon()
			) const
		{
			return
				(  engabra::g3::nearlyEquals(theComps[0], other.theComps[0])
				&& engabra::g3::nearlyEquals(theComps[1], other.theComps[1])
				);
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
			oss
				<< engabra::g3::io::fixed(theComps[0])
				<< ' '
				<< engabra::g3::io::fixed(theComps[1])
				;
			return oss.str();
		}

	}; // Gradel


} // [pix]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::pix::Gradel const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::pix::Gradel const & item
		)
	{
		return item.isValid();
	}

	//! True if both items are same within tol
	inline
	bool
	nearlyEquals
		( quadloco::pix::Gradel const & itemA
		, quadloco::pix::Gradel const & itemB
		, double const & tol = std::numeric_limits<float>::epsilon()
		)
	{
		return itemA.nearlyEquals(itemB, tol);
	}

} // [anon/global]

