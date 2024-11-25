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
 * \brief TODO
 *
 */


#include "imgCircle.hpp"
#include "imgEdgel.hpp"
#include "imgSpot.hpp"

#include <Engabra>

#include <iostream>
#include <numbers>
#include <utility>


namespace quadloco
{

namespace sig
{

	/*! \brief Hough line space "alpha,delta" parameter representation.
	 *
	 * For description, refer to theory/HoughModel.lyx
	 */
	struct ParmAD
	{
		//! Angle position of line segment start point
		double theAlpha{ engabra::g3::null<double>() };

		//! Difference in angular position of line end point from start point
		double theDelta{ engabra::g3::null<double>() };


		//! Point that contains line segmenet (same point as img::Spot)
		inline
		static
		img::Vector<double>
		linePntFromEdgeLoc
			( img::Spot const & spot
			)
		{
			return img::Vector<double>
				{ (double)spot[0]
				, (double)spot[1]
				};
		}

		//! Direction perpendicular to gradient direction in right hand sense
		inline
		static
		img::Vector<double>
		lineDirFromEdgeDir
			( img::Grad const & grad
			)
		{
			return img::Vector<double>
				{ -(double)grad[1]
				,  (double)grad[0]
				};
		}

		/*! \brief Half-open interval enforcement around std::atan2().
		 *
		 * Ensure that return values is in the strict half open interval
		 * \arg -pi <= result < pi
		 */
		/*
		inline
		static
		double
		arcTan2
			( double const & dy
			, double const & dx
			)
		{
			double result{ std::atan2(dy, dx) };
			constexpr double pi{ std::numbers::pi_v<double> };
			if (pi == result)
			{
				result = -pi;
			}
			return result;
		}
		*/

		//! Angle (from circle center) to line seg start on cicle
		inline
		static
		double
		alphaFor
			( img::Spot const & spotOnCircle
			, img::Circle const & circle
			)
		{
			double const dx{ spotOnCircle[0] - circle.theCenter[0] };
			double const dy{ spotOnCircle[1] - circle.theCenter[1] };
			double const alpha{ std::atan2(dy, dx) };
			return alpha;
		}

		//! Angle (from circle center) to line seg end on cicle
		inline
		static
		double
		deltaFor
			( img::Spot const & spotOnCircle
			, img::Circle const & circle
			, double const & alpha
			)
		{
			double const dx{ spotOnCircle[0] - circle.theCenter[0] };
			double const dy{ spotOnCircle[1] - circle.theCenter[1] };
			double delta{ (std::atan2(dy, dx) - alpha) };
			constexpr double piTwo{ 2.*std::numbers::pi_v<double> };
			if (delta < 0.)
			{
				delta = delta + piTwo;
			}
			// this check shouldn't be necessary, but for safety
			if (! (delta < piTwo))
			{
				delta = delta - piTwo;
			}
			return delta;
		}

		inline
		static
		ParmAD
		from
			( img::Edgel const & edgel
			, img::Circle const & circle
			)
		{
			// compute intersection points on circle
			img::CircleIntersector const intersector{ circle };
			std::pair<img::Spot, img::Spot> const solnPair
				{ intersector
					( linePntFromEdgeLoc(edgel.location())
					, lineDirFromEdgeDir(edgel.gradient())
					)
				};

			// compute alpha,delta values for intersection points
			double const alpha{ alphaFor(solnPair.first, circle) };
			double const delta{ deltaFor(solnPair.second, circle, alpha) };
			ParmAD const parmAD{ alpha, delta };

			return parmAD;
		}

		//! True if this instance is valid
		inline
		bool
		isValid
			() const
		{
			return
				(  engabra::g3::isValid(theAlpha)
				&& engabra::g3::isValid(theDelta)
				);
		}

		//! Direct access to theAlpha
		inline
		double const &
		alpha
			() const
		{
			return theAlpha;
		}

		//! Direct access to theDelta
		inline
		double const &
		delta
			() const
		{
			return theDelta;
		}

		//! True if this instance is nearly the same as other within tol
		inline
		bool
		nearlyEquals
			( ParmAD const & other
			, double const & tol = std::numeric_limits<double>::epsilon()
			) const
		{
			// The alpha value testing needs to consider wrap around
			// e.g. for alpha values near +/- pi
			// Unitary magnitude vectors: angle tolerance ~= vector tolerance
			img::Spot const spotA
				{ std::cos(theAlpha)
				, std::sin(theAlpha)
				};
			img::Spot const spotB
				{ std::cos(other.theAlpha)
				, std::sin(other.theAlpha)
				};
			return
				(  spotA.nearlyEquals(spotB, tol)
				&& engabra::g3::nearlyEqualsAbs(theDelta, other.theDelta, tol)
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
			using engabra::g3::io::fixed;
			oss
				<< "theAlpha: " << fixed(theAlpha)
				<< ' '
				<< "theDelta: " << fixed(theDelta)
				;

			return oss.str();
		}

	}; // ParmAD


} // [sig]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::sig::ParmAD const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::sig::ParmAD const & item
		)
	{
		return item.isValid();
	}

	//! True if both items have very nearly the same values
	inline
	bool
	nearlyEquals
		( quadloco::sig::ParmAD const & itemA
		, quadloco::sig::ParmAD const & itemB
		, double const & tol = std::numeric_limits<double>::epsilon()
		)
	{
		return itemA.nearlyEquals(itemB, tol);
	}

} // [anon/global]

