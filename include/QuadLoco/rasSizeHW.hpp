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
 * \brief Structures for raster area management
 *
 */


#include "QuadLoco/imgSpot.hpp"

#include <array>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>


namespace quadloco
{

namespace ras
{

	//! Simple container for 2D raster area dimensions (high & wide)
	class SizeHW
	{
		std::array<std::size_t, 2u> theHighWide{ 0u, 0u };

	public: // methods

		//! default null constructor
		inline
		SizeHW
			() = default;

		//! value construction
		constexpr
		inline
		explicit
		SizeHW
			( std::size_t const & high
			, std::size_t const & wide
			)
			: theHighWide{ high, wide }
		{ }

		//! Check if instance is valid (has a non-zero area)
		inline
		bool
		isValid
			() const
		{
			return (0u < size());
		}

		//! Index for ([0]:high, [1]:wide) !! No bounds checking!!
		inline
		std::size_t const &
		operator[]
			( std::size_t const & ndx
			) const
		{
			return theHighWide[ndx];
		}

		//! Height
		inline
		std::size_t const &
		high
			() const
		{
			return theHighWide[0];
		}

		//! Width
		inline
		std::size_t const &
		wide
			() const
		{
			return theHighWide[1];
		}

		//! Size (high() * wide())
		inline
		std::size_t
		size
			() const
		{
			return (theHighWide[0] * theHighWide[1]);
		}

		//! Length of perimeter in cells = 2u*(high()+wide())
		inline
		std::size_t
		perimeter
			() const
		{
			return (2u * (high() + wide()));
		}

		//! Subpixel precise length of the diagonal
		inline
		double
		diagonal
			() const
		{
			double const diag{ std::hypot((double)high(), (double)wide()) };
			return diag;
		}

		//! Subpixel precise location of hwSize's center (in row,col frame)
		inline
		img::Spot
		centerSpot
			() const
		{
			return img::Spot
				{ .5 * (double)high()
				, .5 * (double)wide()
				};
		}

		//! True if all dimensions are exactly equal
		inline
		bool
		operator==
			( SizeHW const & other
			) const
		{
			return
				(  (other.theHighWide[0] == theHighWide[0])
				&& (other.theHighWide[1] == theHighWide[1])
				);
		}

		//! Descriptive information about this instance.
		inline
		std::string
		infoString
			( std::string const & title = std::string()
			) const
		{
			std::ostringstream oss;
			if (! title.empty())
			{
				oss << title << ' ';
			}
			oss
				<< "high: " << std::setw(5u) << high()
				<< ' ' 
				<< "wide: " << std::setw(5u) << wide()
				;
			return oss.str();
		}
	};

} // [ras]

} // [quadloco]

namespace
{
	//! Put obj.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::ras::SizeHW const & obj
		)
	{
		ostrm << obj.infoString();
		return ostrm;
	}

} // [anon/global]

