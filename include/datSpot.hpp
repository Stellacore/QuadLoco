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


#include "datVec2D.hpp"

// #include <Engabra>

#include <iostream>
// #include <array>
// #include <cmath>
// #include <limits>
// #include <sstream>
// #include <string>


namespace quadloco
{

namespace dat
{

	//! Discrete grid location in row,colum order.
	struct Spot : public dat::Vec2D<double>
	{
		inline
		explicit
		Spot
			()
			: dat::Vec2D<double>{}
		{ }

		inline
		explicit
		Spot
			( double const val0
			, double const val1
			)
			: dat::Vec2D<double>{ val0, val1 }
		{ }

		inline
		explicit
		Spot
			( dat::Vec2D<double> const vec2D
			)
			: dat::Vec2D<double>{ vec2D.theData }
		{ }

		inline
		virtual
		~Spot
			() = default;

		//! Alias for (*this)[0]
		inline
		double const &
		xVal
			() const
		{
			return (*this)[0];
		}

		//! Alias for (*this)[1]
		inline
		double const &
		yVal
			() const
		{
			return (*this)[1];
		}

		//! Alias for (*this)[0]
		inline
		double const &
		row
			() const
		{
			return (*this)[0];
		}

		//! Alias for (*this)[1]
		inline
		double const &
		col
			() const
		{
			return (*this)[1];
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

} // [anon/global]

