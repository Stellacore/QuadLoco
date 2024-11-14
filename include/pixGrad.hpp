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
 * \brief Declarations for quadloco::pix::Grad
 *
 */


#include "datVec2D.hpp"

#include <iostream>


#include <format>
namespace quadloco
{

namespace pix
{

	/*! \brief GRADent ELement structure representing a directed edge gradient.
	 *
	 * \note For computing gradient values over entire dat::Grid instances
	 * refer to functions in pixgrid.hpp (e.g. gradientGridFor()).
	 */
	struct Grad : public dat::Vec2D<float>
	{

		//! Functor for formatting Grad data into a string
		struct Formatter
		{
			//! std::format to apply to each of the two theComps
			std::string const theFormatEach{ "{:4.1f}" };

			inline
			std::string
			operator()
				( Grad const & elem
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

		inline
		explicit
		Grad
			()
			: dat::Vec2D<float>{}
		{ }

		inline
		explicit
		Grad
			( float const val0
			, float const val1
			)
			: dat::Vec2D<float>{ val0, val1 }
		{ }

		inline
		explicit
		Grad
			( dat::Vec2D<float> const vec2D
			)
			: dat::Vec2D<float>{ vec2D.theData }
		{ }

		inline
		virtual
		~Grad
			() = default;


		//! Alias for (*this)[0]
		inline
		float const &
		dx
			() const
		{
			return (*this)[0];
		}

		//! Alias for (*this)[1]
		inline
		float const &
		dy
			() const
		{
			return (*this)[1];
		}

		//! Alias for (*this)[0]
		inline
		float const &
		drow
			() const
		{
			return (*this)[0];
		}

		//! Alias for (*this)[1]
		inline
		float const &
		dcol
			() const
		{
			return (*this)[1];
		}


	}; // Grad


} // [pix]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::pix::Grad const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::pix::Grad const & item
		)
	{
		return item.isValid();
	}

	//! True if both items are same within tol
	inline
	bool
	nearlyEquals
		( quadloco::pix::Grad const & itemA
		, quadloco::pix::Grad const & itemB
		, float const & tol = std::numeric_limits<float>::epsilon()
		)
	{
		return itemA.nearlyEquals(itemB, tol);
	}

} // [anon/global]

