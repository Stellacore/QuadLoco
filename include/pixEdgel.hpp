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
 * \brief Declarations for quadloco::pix::Edgel
 *
 */


#include "cast.hpp"
#include "pixGrad.hpp"
#include "pixSpot.hpp"

#include <Engabra>

#include <limits>
#include <sstream>
#include <string>


namespace quadloco
{

namespace pix
{

	//! Edge element in raster space (location and gradient)
	struct Edgel
	{
		//! Any point on the line
		pix::Spot const theSpot{};

		//! Direction of the (positive) gradient across the edge
		pix::Grad const theGrad{};


		//! Location of this edgel
		inline
		pix::Spot const &
		location
			() const
		{
			return theSpot;
		}

		//! Gradient at this edgel location
		inline
		pix::Grad const &
		gradient
			() const
		{
			return theGrad;
		}

		//! True if both the point location and gradent direction are valid
		inline
		bool
		isValid
			() const
		{
			return
				(  location().isValid()
				&& gradient().isValid()
				);
		}

		//! True if location is in front of edge (relative to gradient)
		inline
		bool
		rcInFront
			( dat::Spot const & rcDatSpot
			) const
		{
			return (! rcInBack(rcDatSpot));
		}

		//! True if location is behind the edge (relative to gradient)
		inline
		bool
		rcInBack
			( dat::Spot const & rcDatSpot
			) const
		{
			return rcInBack
				(pix::Spot{ (float)rcDatSpot.row(), (float)rcDatSpot.col() });
		}

		//! True if location is in front of edge (relative to gradient)
		inline
		bool
		rcInFront
			( pix::Spot const & rcPixSpot
			) const
		{
			return rcInFront	
				(dat::Spot{ (double)rcPixSpot.row(), (double)rcPixSpot.col() });
		}

		//! True if location is behind the edge (relative to gradient)
		inline
		bool
		rcInBack
			( pix::Spot const & rcPixSpot
			) const
		{
			pix::Spot const delta{ rcPixSpot - location() };
			float const projection{ dot(delta, gradient()) };
			return (projection < 0.f);
		}

		//! True if location is in front of edge (relative to gradient)
		inline
		bool
		rcInFront
			( dat::RowCol const & rowcol
			) const
		{
			return rcInFront	
				(pix::Spot{ (float)rowcol.row(), (float)rowcol.col() });
		}

		//! True if location is behind the edge (relative to gradient)
		inline
		bool
		rcInBack
			( dat::RowCol const & rowcol
			) const
		{
			return rcInBack
				(pix::Spot{ (float)rowcol.row(), (float)rowcol.col() });
		}

		//! True if components are same as those of other within tol
		inline
		bool
		nearlyEquals
			( Edgel const & other
			, double const & tol = std::numeric_limits<double>::epsilon()
			) const
		{
			return
				(  location().nearlyEquals(other.location())
				&& gradient().nearlyEquals(other.gradient())
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
				<< "location(): " << location()
				<< ' '
				<< "gradient(): " << gradient()
				;

			return oss.str();
		}

	}; // Edgel

} // [pix]

} // [quadloco]

namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::pix::Edgel const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::pix::Edgel const & item
		)
	{
		return item.isValid();
	}

	//! True if both items have very nearly the same values
	inline
	bool
	nearlyEquals
		( quadloco::pix::Edgel const & itemA
		, quadloco::pix::Edgel const & itemB
		, double const & tol = std::numeric_limits<double>::epsilon()
		)
	{
		return itemA.nearlyEquals(itemB, tol);
	}

} // [anon/global]
