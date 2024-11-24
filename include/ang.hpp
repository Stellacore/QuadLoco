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
 * \brief Declarations for angle related functions in quadloco::ang
 *
 */


#include <Engabra>

#include <cmath>
#include <limits>
#include <numbers>



namespace quadloco
{

/*! \brief Angle related function namespace quadloco::ang
 */
namespace ang
{

	//! Angle value for a full rotation
	inline
	constexpr
	double
	piOne
		()
	{
		return (std::numbers::pi_v<double>);
	}

	//! Angle value for a full rotation
	inline
	constexpr
	double
	piTwo
		()
	{
		return (2. * std::numbers::pi_v<double>);
	}

	//! Wrapper on std::atan2() that returns half open interval [-pi,+pi)
	inline
	double
	atan2
		( double const & yy
		, double const & xx
		)
	{
		double angle(std::atan2(yy, xx));
		// compare (using value computed with (hopefully) the same function
		static double const wrapPoint(std::atan2( 0., -1.)); // +pi
		///    double const wrapPoint(std::atan2(-0., -1.)); // -pi
		if (wrapPoint == angle)
		{
			angle = -wrapPoint;
		}
		return angle;
	}

	//! Angle within half open principal range [-pi,+pi)
	inline
	double
	principalAngle
		( double const & anyAngle
		)
	{
		// note: std::atan2 can return either -pi or +pi !! so use above one
		return (atan2(std::sin(anyAngle), std::cos(anyAngle)));
	}

	//! Angle that is in the half open positive range [0,+2pi)
	inline
	double
	nonNegativeAngle
		( double const & anyAngle
		)
	{
		double angle(principalAngle(anyAngle));
		if (angle < 0.)
		{
			angle += piTwo();
		}
		return angle;
	}

	//! True if angles 1 and 2 are nearly at same location on unit circle
	inline
	bool
	nearlySameAngle
		( double const & angle1
		, double const & angle2
		, double const & tol = std::numeric_limits<double>::epsilon()
		)
	{
		double const c1{ std::cos(angle1) };
		double const c2{ std::cos(angle2) };
		double const s1{ std::sin(angle1) };
		double const s2{ std::sin(angle2) };
		return
			(  engabra::g3::nearlyEqualsAbs(c1, c2, tol)
			&& engabra::g3::nearlyEqualsAbs(s1, s2, tol)
			);
	}

} // [ang]

} // [quadloco]

