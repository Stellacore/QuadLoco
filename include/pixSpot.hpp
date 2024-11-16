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
 * \brief Declarations for quadloc::pix::Spot - floating point 2D locations
 *
 */


#include "datVec2D.hpp"

#include <iostream>


namespace quadloco
{

namespace pix
{

	//! Discrete grid location in row,colum order.
	struct Spot : public dat::Vec2D<float>
	{
		inline
		explicit
		Spot
			()
			: dat::Vec2D<float>{}
		{ }

		inline
		explicit
		Spot
			( float const val0
			, float const val1
			)
			: dat::Vec2D<float>{ val0, val1 }
		{ }

		inline
		explicit
		Spot
			( dat::Vec2D<float> const vec2D
			)
			: dat::Vec2D<float>{ vec2D.theData }
		{ }

		inline
		virtual
		~Spot
			() = default;

		//! Alias for (*this)[0]
		inline
		float const &
		row
			() const
		{
			return (*this)[0];
		}

		//! Alias for (*this)[1]
		inline
		float const &
		col
			() const
		{
			return (*this)[1];
		}

	}; // Spot


} // [pix]

} // [quadloco]


namespace
{
	//! Put obj.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::pix::Spot const & obj
		)
	{
		ostrm << obj.infoString();
		return ostrm;
	}

	//! True if spot is not null
	inline
	bool
	isValid
		( quadloco::pix::Spot const & spot
		)
	{
		return spot.isValid();
	}

	//! Are coordinates of each spot numerically same within tolerance?
	inline
	bool
	nearlyEquals
		( quadloco::pix::Spot const & spotA
		, quadloco::pix::Spot const & spotB
		, float const & tol = std::numeric_limits<float>::epsilon()
		)
	{
		return spotA.nearlyEquals(spotB, tol);
	}

} // [anon/global]

