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
 * \brief Declarations for img::Spot
 *
 */


#include "imgVector.hpp"

// #include <Engabra>

#include <iostream>
// #include <array>
// #include <cmath>
// #include <limits>
// #include <sstream>
// #include <string>


namespace quadloco
{

namespace img
{

	//! Discrete grid location in row,colum order.
	struct Spot : public img::Vector<double>
	{
		//! Default construction of a null instance (isValid() == false)
		inline
		explicit
		Spot
			()
			: img::Vector<double>{}
		{ }

		//! Explicit value construction.
		inline
		explicit
		Spot
			( double const val0
			, double const val1
			)
			: img::Vector<double>{ val0, val1 }
		{ }

		//! Construct with values after cast to double type
		inline
		explicit
		Spot
			( std::size_t const val0
			, std::size_t const val1
			)
			: img::Vector<double>
				{ static_cast<double>(val0)
				, static_cast<double>(val1)
				}
		{ }

		//! Construct from baseclass instance.
		inline
		explicit
		Spot
			( img::Vector<double> const vec2D
			)
			: img::Vector<double>{ vec2D.theData }
		{ }

		//! Noop dtor.
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


} // [img]

} // [quadloco]


namespace
{
	//! Put obj.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::img::Spot const & obj
		)
	{
		ostrm << obj.infoString();
		return ostrm;
	}

	//! True if spot is not null
	inline
	bool
	isValid
		( quadloco::img::Spot const & spot
		)
	{
		return spot.isValid();
	}

	//! Are coordinates of each spot numerically same within tolerance?
	inline
	bool
	nearlyEquals
		( quadloco::img::Spot const & spotA
		, quadloco::img::Spot const & spotB
		, double const & tol = std::numeric_limits<double>::epsilon()
		)
	{
		return spotA.nearlyEquals(spotB, tol);
	}

} // [anon/global]

