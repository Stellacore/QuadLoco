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
		std::array<double, 2u> theLocRC;

		//! True if individual coordinates of are numerically same within tol
		inline
		bool
		nearlyEquals
			( Spot const & other
			, double const & tol = std::numeric_limits<double>::epsilon()
			) const
		{
			double const d0{ (other.theLocRC[0] - theLocRC[0]) };
			double const d1{ (other.theLocRC[1] - theLocRC[1]) };
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
			oss
				<< fixed(theLocRC[0])
				<< ' ' << fixed(theLocRC[1])
				;
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

