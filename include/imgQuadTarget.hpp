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
 * \brief Declarations for quadloco::img::QuadTarget
 *
 */


#include <Engabra>

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>


namespace quadloco
{

namespace img
{
	//! Geometric description for perspective image of an obj::QuadTarget
	struct QuadTarget
	{
		// minimum image information for saying this is a quad target
		// (e.g. all outer edges could be cropped from image sample)

		//! \brief Center is required
		engabra::g3::Vector const theCenter
			{engabra::g3::null<engabra::g3::Vector>()};

		/*! \brief One of two *UNITARY* directions - target "x-axis".
		 *
		 * This direction is associated with one of the two radial
		 * edges that has foreground (white) to the right side of
		 * the edge, and background (dark) to the left of the edge.
		 */
		engabra::g3::Vector const theDirX
			{engabra::g3::null<engabra::g3::Vector>()};


		//! \brief Second of two *UNITARY* directions - target "y-axis".
		engabra::g3::Vector const theDirY
			{engabra::g3::null<engabra::g3::Vector>()};


		//! True if this instance contains valid data (not null)
		inline
		bool
		isValid
			() const
		{
			return
				(  engabra::g3::isValid(theCenter)
				&& engabra::g3::isValid(theDirX)
				&& engabra::g3::isValid(theDirY)
				);
		}

		//! True if theDirX(wedge)theDirY is aligned with e12
		inline
		bool
		isDextral
			() const
		{
			using namespace engabra::g3;
			BiVector const bivXY{ (theDirX * theDirY).theBiv };
			double const dot{ (-bivXY * e12).theSca[0] };
			return (0. < dot);
		}

		//! True if this instance isValid() with (tol<angleSizeYwX)
		inline
		bool
		isStable
			( double const tol = std::numeric_limits<double>::epsilon()
			) const
		{
			bool stable{ isValid() };
			if (stable)
			{
				stable &= (tol < std::abs(angleSizeYwX()));
			}
			return stable;
		}

		//! Scalar (signed) angle from theDirX toward theDirY
		inline
		double
		angleSizeYwX
			() const
		{
			using namespace engabra::g3;
			Spinor const spinYwX{ direction(theDirX) * direction(theDirY) };
			BiVector const angle{ logG2(spinYwX).theBiv };
			double const angSize{ -(angle * e12).theSca[0] };
			return angSize;
		}

		//! True if this is same as other within numeric tolerance
		inline
		bool
		nearlyEquals
			( quadloco::img::QuadTarget const & other
			, double const tol = std::numeric_limits<double>::epsilon()
			) const
		{
			bool same{ isValid() && other.isValid() };
			if (same)
			{
				using engabra::g3::nearlyEquals;
				// centers need to be the same
				bool const okayCenter
					{ nearlyEquals(theCenter, other.theCenter, tol) };

				// both directions need to be parallel or anti-parallel
				bool const okayPos
					{  nearlyEquals(theDirX,  other.theDirX, tol)
					&& nearlyEquals(theDirY,  other.theDirY, tol)
					};
				bool const okayNeg
					{  nearlyEquals(theDirX, -other.theDirX, tol)
					&& nearlyEquals(theDirY, -other.theDirY, tol)
					};
				bool const okayDir{ okayPos || okayNeg };

				// center *and* one of direction conventions
				same = okayCenter && okayDir;
			}
			return same;
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
				oss << title << '\n';
			}
			oss
				<< "isValid: " << std::boolalpha << isValid()
					<< "  isStable: " << std::boolalpha << isStable()
					<< "  isDextral: " << std::boolalpha << isDextral()
				<< '\n'
				<< "center(r,c): " << theCenter
				<< '\n'
				<< "  dirX(r,c): " << theDirX
				<< '\n'
				<< "  dirY(r,c): " << theDirY
				;
			return oss.str();
		}

	}; // QuadTarget


} // [img]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::img::QuadTarget const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::img::QuadTarget const & item
		)
	{
		return item.isValid();
	}

	//! True if itemA and itemB are same within tolerance
	inline
	bool
	nearlyEquals
		( quadloco::img::QuadTarget const & itemA
		, quadloco::img::QuadTarget const & itemB
		, double const tol = std::numeric_limits<double>::epsilon()
		)
	{
		return itemA.nearlyEquals(itemB, tol);
	}


} // [anon/global]

