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
 * \brief Declarations for quadloco::img::Grad
 *
 */


#include "imgVector.hpp"

#include <iostream>


namespace quadloco
{

namespace img
{

	/*! \brief GRADent ELement structure representing a directed edge gradient.
	 *
	 * \note For computing gradient values over entire ras::Grid instances
	 * refer to functions in rasgrid.hpp (e.g. gradientGridFor()).
	 */
	struct Grad : public img::Vector<double>
	{

		inline
		explicit
		Grad
			()
			: img::Vector<double>{}
		{ }

		inline
		explicit
		Grad
			( double const val0
			, double const val1
			)
			: img::Vector<double>{ val0, val1 }
		{ }

		inline
		explicit
		Grad
			( img::Vector<double> const vec2D
			)
			: img::Vector<double>{ vec2D.theData }
		{ }

		inline
		virtual
		~Grad
			() = default;


		//! Alias for (*this)[0]
		inline
		double const &
		dx
			() const
		{
			return (*this)[0];
		}

		//! Alias for (*this)[1]
		inline
		double const &
		dy
			() const
		{
			return (*this)[1];
		}

		//! Alias for (*this)[0]
		inline
		double const &
		drow
			() const
		{
			return (*this)[0];
		}

		//! Alias for (*this)[1]
		inline
		double const &
		dcol
			() const
		{
			return (*this)[1];
		}


	}; // Grad


} // [img]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::img::Grad const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::img::Grad const & item
		)
	{
		return item.isValid();
	}

	//! True if both items are same within tol
	inline
	bool
	nearlyEquals
		( quadloco::img::Grad const & itemA
		, quadloco::img::Grad const & itemB
		, double const & tol = std::numeric_limits<double>::epsilon()
		)
	{
		return itemA.nearlyEquals(itemB, tol);
	}

} // [anon/global]

