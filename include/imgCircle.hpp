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
 * \brief Declaration for quadloco::img::Circle
 *
 */


#include "imgSpot.hpp"
#include "imgVector.hpp"

#include <Engabra>

#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>


namespace quadloco
{

namespace img
{

	//! Circle containing bounding rectangle
	struct Circle
	{
		//! Circle center location (in [pix])
		img::Spot const theCenter{};

		//! Circle radius (in [pix])
		double const theRadius{ engabra::g3::null<double>() };

		//! Circle circumscribing format
		inline
		static
		Circle
		circumScribing
			( std::size_t const & high
			, std::size_t const & wide
			)
		{
			img::Spot const corner
				{ static_cast<double>(high)
				, static_cast<double>(wide)
				};
			img::Spot const center{ .5 * corner };
			double const radius{ .5 * magnitude(corner) };
			return Circle{ center, radius };
		}

		//! True if this instance is valid
		inline
		bool
		isValid
			() const
		{
			return
				(  theCenter.isValid()
				&& engabra::g3::isValid(theRadius)
				);
		}

		//! True if this instance is nearly the same as other within tol
		inline
		bool
		nearlyEquals
			( Circle const & other
			, double const & tol = std::numeric_limits<double>::epsilon()
			) const
		{
			return
				(  theCenter.nearlyEquals(other.theCenter, tol)
				&& engabra::g3::nearlyEquals(theRadius, other.theRadius, tol)
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
				<< "theCenter: " << theCenter
				<< ' '
				<< "theRadius: " << theRadius
				;
			return oss.str();
		}

	}; // Circle


	//! Functor for returning intersection of a line with a circle
	struct CircleIntersector
	{
		Circle const theCircle{};


		/*! \brief Pair of solutions at sides of edges that intersect circle.
		 *
		 * NOTE: The edgel defines a location and intensity gradient, g.
		 *       This function defines a line direction *PERPENDICULAR* to
		 *       the gradient direction (in right hand sense: g is rotated
		 *       a quarter turn into line direction).
		 */
		inline
		std::pair<img::Spot, img::Spot>
		operator()
			( img::Vector<double> const & linePnt
				//!< Any arbirary point on the line
			, img::Vector<double> const & lineDir
				//!< Positive direction along line
			) const
		{
			using img::Spot;
			std::pair<Spot, Spot> solnPair{ Spot{}, Spot{} };

			// use engabra vectors for coding convenience
			using namespace engabra::g3;

			// be sure direction is unitary (supresses quadratic coefficient)
			img::Vector<double> const ddir{ direction(lineDir) };
			img::Vector<double> const & spnt = linePnt;

			img::Vector<double> const cpnt{ theCircle.theCenter };
			double const & rho = theCircle.theRadius;

			// quadratic equation components
			img::Vector<double> const wvec{ spnt - cpnt };
			double const beta{ dot(wvec, ddir) };
			double const wMagSq{ dot(wvec, wvec) };
			double const gamma{ wMagSq - rho*rho };
			double const lamMid{ -beta };
			double const radicand{ beta*beta - gamma };

			// check for real roots (else return default null instances)
			if (! (radicand < 0.))
			{
				double const delta{ std::sqrt(radicand) };
				double const lamNeg{ lamMid - delta };
				double const lamPos{ lamMid + delta };

				img::Vector<double> const xNeg{ spnt + lamNeg*ddir };
				img::Vector<double> const xPos{ spnt + lamPos*ddir };

				solnPair.first  = img::Spot(xNeg);
				solnPair.second = img::Spot(xPos);

				/*
				Vector const rPos{ xPos - cpnt };
				Vector const rNeg{ xNeg - cpnt };
				double const magPos{ magnitude(rPos) };
				double const magNeg{ magnitude(rNeg) };
				std::cout << "rPos: " << rPos << '\n';
				std::cout << "rNeg: " << rNeg << '\n';
				std::cout << "magPos: " << magPos << '\n';
				std::cout << "magNeg: " << magNeg << '\n';
				*/
			}

			return solnPair;
		}


	}; // CircleIntersector


} // [img]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::img::Circle const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::img::Circle const & item
		)
	{
		return item.isValid();
	}

	//! True if both items have very nearly the same values
	inline
	bool
	nearlyEquals
		( quadloco::img::Circle const & itemA
		, quadloco::img::Circle const & itemB
		, double const & tol = std::numeric_limits<double>::epsilon()
		)
	{
		return itemA.nearlyEquals(itemB, tol);
	}

} // [anon/global]

