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
 * \brief Declarations for quadloco::img::Edgel
 *
 */


#include "cast.hpp"
#include "imgGrad.hpp"
#include "pixSpot.hpp"

#include <Engabra>

#include <limits>
#include <sstream>
#include <string>


namespace quadloco
{

namespace img
{

	//! Edge element in raster space (location and gradient)
	class Edgel
	{
		//! Any point on the line
		pix::Spot theSpot{};

		//! Direction of the (positive) gradient across the edge
		img::Grad theGrad{};

	public:

		//! Constuct invalid (null) instance
		inline
		explicit
		Edgel
			() = default;

		//! Value construction
		inline
		explicit
		Edgel
			( pix::Spot const & spot
			, img::Grad const & grad
			)
			: theSpot{ spot }
			, theGrad{ grad }
		{ }

		//! Conversion from RowCol
		inline
		explicit
		Edgel
			( ras::RowCol const & rowcol
			, img::Grad const & grad
			)
			: theSpot{ cast::pixSpot(rowcol) }
			, theGrad{ grad }
		{ }

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
		img::Grad const &
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
			( img::Spot const & rcDatSpot
			) const
		{
			return (! rcInBack(rcDatSpot));
		}

		//! True if location is behind the edge (relative to gradient)
		inline
		bool
		rcInBack
			( img::Spot const & rcDatSpot
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
				(img::Spot{ (double)rcPixSpot.row(), (double)rcPixSpot.col() });
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
			( ras::RowCol const & rowcol
			) const
		{
			return rcInFront	
				(pix::Spot{ (float)rowcol.row(), (float)rowcol.col() });
		}

		//! True if location is behind the edge (relative to gradient)
		inline
		bool
		rcInBack
			( ras::RowCol const & rowcol
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
				(  location().nearlyEquals(other.location(), tol)
				&& gradient().nearlyEquals(other.gradient(), tol)
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

} // [img]

} // [quadloco]

namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::img::Edgel const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::img::Edgel const & item
		)
	{
		return item.isValid();
	}

	//! True if both items have very nearly the same values
	inline
	bool
	nearlyEquals
		( quadloco::img::Edgel const & itemA
		, quadloco::img::Edgel const & itemB
		, double const & tol = std::numeric_limits<double>::epsilon()
		)
	{
		return itemA.nearlyEquals(itemB, tol);
	}

} // [anon/global]
