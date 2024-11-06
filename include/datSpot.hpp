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
 * \brief Declarations for dat::Spot
 *
 */


#include <Engabra>

#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>


namespace quadloco
{

namespace dat
{

	//! Discrete grid location in row,colum order.
	struct Spot
	{
		std::array<double, 2u> theLoc
			{ engabra::g3::null<double>()
			, engabra::g3::null<double>()
			};

		//! True if both coordinates are not null.
		inline
		bool
		isValid
			() const
		{
			return
				(  engabra::g3::isValid(theLoc[0])
				&& engabra::g3::isValid(theLoc[1])
				);
		}

		//! Subscript access to locations -- NO bounds checking
		inline
		double const &
		operator[]
			( std::size_t const & ndx
			) const
		{
			return theLoc[ndx];
		}

		//! Alias for #theLoc[0]
		inline
		double const &
		xVal
			() const
		{
			return theLoc[0];
		}

		//! Alias for #theLoc[1]
		inline
		double const &
		yVal
			() const
		{
			return theLoc[1];
		}

		//! Alias for #theLoc[0]
		inline
		double const &
		row
			() const
		{
			return theLoc[0];
		}

		//! Alias for #theLoc[1]
		inline
		double const &
		col
			() const
		{
			return theLoc[1];
		}

		//! True if individual coordinates of are numerically same within tol
		inline
		bool
		nearlyEquals
			( Spot const & other
			, double const & tol = std::numeric_limits<double>::epsilon()
			) const
		{
			double const d0{ (other.row() - row()) };
			double const d1{ (other.col() - col()) };
			double const dMaxAbs{ std::max(std::abs(d0), std::abs(d1)) };
			return (dMaxAbs < tol);
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
			using engabra::g3::io::fixed;
			oss << fixed(row()) << ' ' << fixed(col()) ;
			return oss.str();
		}

	}; // Spot


} // [dat]

} // [quadloco]


namespace
{
	//! Put obj.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::dat::Spot const & obj
		)
	{
		ostrm << obj.infoString();
		return ostrm;
	}

	//! True if spot is not null
	inline
	bool
	isValid
		( quadloco::dat::Spot const & spot
		)
	{
		return spot.isValid();
	}

	//! Are coordinates of each spot numerically same within tolerance?
	inline
	bool
	nearlyEquals
		( quadloco::dat::Spot const & spotA
		, quadloco::dat::Spot const & spotB
		, double const & tol = std::numeric_limits<double>::epsilon()
		)
	{
		return spotA.nearlyEquals(spotB);
	}

	//! Sum of two spot locations
	inline
	quadloco::dat::Spot
	operator+
		( quadloco::dat::Spot const & spotA
		, quadloco::dat::Spot const & spotB
		)
	{
		return quadloco::dat::Spot
			{ (spotA.row() + spotB.row())
			, (spotA.col() + spotB.col())
			};
	}

	//! Difference of two spot locations
	inline
	quadloco::dat::Spot
	operator-
		( quadloco::dat::Spot const & spotA
		, quadloco::dat::Spot const & spotB
		)
	{
		return quadloco::dat::Spot
			{ (spotA.row() - spotB.row())
			, (spotA.col() - spotB.col())
			};
	}

	//! Scalar multiple of spot coordinates
	inline
	quadloco::dat::Spot
	operator*
		( double const & scalar
		, quadloco::dat::Spot const & spot
		)
	{
		return quadloco::dat::Spot
			{ (scalar * spot.row())
			, (scalar * spot.col())
			};
	}

	//! Distance between two spot locations
	inline
	double
	magnitude
		( quadloco::dat::Spot const spot
		)
	{
		return std::hypot(spot.theLoc[0], spot.theLoc[1]);
	}

} // [anon/global]

