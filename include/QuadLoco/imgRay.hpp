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
 * \brief Declarations for quadloco::img::Ray
 *
 */


#include "QuadLoco/imgVector.hpp"

#include <Engabra>

#include <algorithm>
#include <limits>
#include <sstream>
#include <string>
#include <vector>


namespace quadloco
{

namespace img
{

	//! Edge element in raster space (start and direction)
	class Ray
	{
		//! Any point on the line
		img::Vector<double> theStart{};

		//! Positive direction of ray extension from theStart point
		img::Vector<double> theDir{};

	public:

		//! Constuct invalid (null) instance
		inline
		explicit
		Ray
			() = default;

		//! Value construction
		inline
		explicit
		Ray
			( img::Vector<double> const & start
			, img::Vector<double> const & dirVec
			)
			: theStart{ start }
			, theDir{ quadloco::img::direction(dirVec) }
		{ }


		//! True if both the start point and direction are valid
		inline
		bool
		isValid
			() const
		{
			return
				(  start().isValid()
				&& direction().isValid()
				);
		}

		//! Start point on this ray
		inline
		img::Vector<double> const &
		start
			() const
		{
			return theStart;
		}

		//! Forward direction of this ray
		inline
		img::Vector<double> const &
		direction
			() const
		{
			return theDir;
		}

		//! Direction orthogonal (perpendicular) to direction() in R.H. sense
		inline
		img::Vector<double>
		orthoDirection
			() const
		{
			// direction after a quarter turn rotation in 2D
			return ccwPerp(theDir);
		}

		//! Distance along ray: from start point to projectionOf()
		inline
		double
		distanceAlong
			( img::Vector<double> const & anyPnt
			) const
		{
			return dot((anyPnt - start()), theDir);
		}

		//! Distance of closest approach to ray
		inline
		double
		distanceFrom
			( img::Vector<double> const & anyPnt
			) const
		{
			return outer(theDir, (anyPnt - start()));
		}

		//! Point location on ray that is projection of anyPnt onto ray
		inline
		img::Vector<double>
		projectionOf
			( img::Vector<double> const & anyPnt
			) const
		{
			return (distanceAlong(anyPnt) * theDir);
		}

		//! True if point location is in front of edge (relative to direction)
		inline
		bool
		isAhead
			( img::Vector<double> const & anyPnt
			) const
		{
			return (! isBehind(anyPnt));
		}

		//! True if point location is behind the edge (relative to direction)
		inline
		bool
		isBehind
			( img::Vector<double> const & anyPnt
			) const
		{
			return (distanceAlong(anyPnt) < 0.);
		}

		//! True if components are same as those of other within tol
		inline
		bool
		nearlyEquals
			( Ray const & other
			, double const & tol = std::numeric_limits<double>::epsilon()
			) const
		{
			return
				(  start().nearlyEquals(other.start(), tol)
				&& direction().nearlyEquals(other.direction(), tol)
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
				<< "start(): " << start()
				<< ' '
				<< "direction(): " << direction()
				;

			return oss.str();
		}

	}; // Ray

} // [img]

} // [quadloco]

namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::img::Ray const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::img::Ray const & item
		)
	{
		return item.isValid();
	}

	//! True if both items have very nearly the same values
	inline
	bool
	nearlyEquals
		( quadloco::img::Ray const & itemA
		, quadloco::img::Ray const & itemB
		, double const & tol = std::numeric_limits<double>::epsilon()
		)
	{
		return itemA.nearlyEquals(itemB, tol);
	}

	//! Put collection of edge rays to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, std::vector<quadloco::img::Ray> const & edgeRays
		)
	{
		for (quadloco::img::Ray const & edgeRay : edgeRays)
		{
			ostrm << edgeRay << '\n';
		}
		return ostrm;
	}

	//! True if edgel is colinear with ray within (positional) tolerance
	inline
	static
	bool
	nearlyCollinear
		( quadloco::img::Ray const & ray1
		, quadloco::img::Ray const & ray2
		, double const & tolDelta
		)
	{
		quadloco::img::Vector<double> const & s1 = ray1.start();
		quadloco::img::Vector<double> const & s2 = ray2.start();
		double const dist12{ ray1.distanceAlong(s2) };
		double const dist21{ ray2.distanceAlong(s1) };
		double const distMax
			{ std::max(std::abs(dist12), std::abs(dist21)) };
		return (distMax < tolDelta);
	}


} // [anon/global]
