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
 * \brief Declarations for quadloco::dat::Vec2D
 *
 */


#include <Engabra>

#include <array>
#include <cmath>


namespace quadloco
{

namespace dat
{

	//! Generic 2D vector structure
	struct Vec2D
	{
		//! Component data values
		std::array<double, 2u> theData
			{ engabra::g3::null<double>()
			, engabra::g3::null<double>()
			};

		//! True if this instance contain valid data (is not null)
		inline
		bool
		isValid
			() const
		{
			return engabra::g3::isValid(theData);
		}

		//! Access to component values
		inline
		double const &
		operator[]
			( std::size_t const & ndx
			) const
		{
			return theData[ndx];
		}

		//! True if this is same as other within tolerance
		inline
		bool
		nearlyEquals
			( Vec2D const & other
			, double const tol = std::numeric_limits<double>::epsilon()
			) const
		{
			return
				{  engabra::g3::nearlyEquals(theData[0], other.theData[0])
				&& engabra::g3::nearlyEquals(theData[1], other.theData[1])
				};
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
				<< engabra::g3::io::fixed(theData[0])
				<< ' '
				<< engabra::g3::io::fixed(theData[1])
				;

			return oss.str();
		}

	}; // Vec2D

	//! Scaling operations
	inline
	Vec2D
	operator*
		( double const & scl
		, Vec2D const & vec
		)
	{
		return { scl*vec[0], scl*vec[1] };
	}

	//
	// Vector properties
	//

	inline
	double
	magnitude
		( Vec2D const & vec
		)
	{
		return std::hypot(vec[0], vec[1]);
	}

	inline
	Vec2D
	direction
		( Vec2D const & vec
		)
	{
		Vec2D unit{};
		double const mag{ magnitude(vec) };
		if (engabra::g3::isValid(mag))
		{
			unit = (1./mag) * vec;
		}
		return unit;
	}

	//
	// Binary
	//

	//! Sum (vecA + vecB)
	inline
	Vec2D
	operator+
		( Vec2D const & vecA
		, Vec2D const & vecB
		)
	{
		return Vec2D{ vecA[0] + vecB[0], vecA[1] + vecB[1] };
	}

	//! Difference (vecA - vecB)
	inline
	Vec2D
	operator-
		( Vec2D const & vecA
		, Vec2D const & vecB
		)
	{
		return Vec2D{ vecA[0] - vecB[0], vecA[1] - vecB[1] };
	}


	//! Scalar dot product
	inline
	double
	dot
		( Vec2D const & vecA
		, Vec2D const & vecB
		)
	{
		return (vecA[0]*vecB[0] + vecA[1]*vecB[1]);
	}

	//! Real analog of wedge product
	inline
	double
	outer
		( Vec2D const & vecA
		, Vec2D const & vecB
		)
	{
		return (vecA[0]*vecB[1] - vecA[1]*vecB[0]);
	}

} // [dat]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::dat::Vec2D const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::dat::Vec2D const & item
		)
	{
		return item.isValid();
	}

	//! True if both instances are same within tolerance
	inline
	bool
	nearlyEquals
		( quadloco::dat::Vec2D const & itemA
		, quadloco::dat::Vec2D const & itemB
		, double const tol = std::numeric_limits<double>::epsilon()
		)
	{
		return itemA.nearlyEquals(itemB, tol);
	}

} // [anon/global]

