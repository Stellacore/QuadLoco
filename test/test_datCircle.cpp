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


/*! \file
\brief Unit tests (and example) code for quadloco::dat::Circle
*/


#include "datCircle.hpp"
#include "fndHoughAD.hpp"

#include <iostream>
#include <sstream>


namespace
{
	//! True if loc lies on circle geometry
	inline
	double
	rejectFromCircle
		( quadloco::dat::Spot const & loc
		, quadloco::dat::Circle const & circle
		, double const tol = std::numeric_limits<double>::epsilon()
		)
	{
		double rej{ engabra::g3::null<double>() };
		quadloco::dat::Spot const delta{ loc - circle.theCenter };
		double const magSq
			{ delta[0] * delta[0]
			+ delta[1] * delta[1]
			};
		double const gotRad{ std::sqrt(magSq) };
		rej = gotRad - circle.theRadius;
		return rej;
	}

	//! True if loc lies on line geometry
	inline
	double
	rejectFromLine
		( quadloco::dat::Spot const & loc
		, quadloco::dat::Spot const & linePnt
		, quadloco::dat::Vec2D const & lineDir
		, double const tol = std::numeric_limits<double>::epsilon()
		)
	{
		double rej{ engabra::g3::null<double>() };
		using namespace quadloco;
		dat::Spot const delta{ loc - linePnt };
		dat::Vec2D const unitDir{ direction(lineDir) };
		rej = dat::outer(lineDir, unitDir);
		return rej;
	}


	//! Examples for documentation
	void
	test0
		( std::ostream & oss
		)
	{
		quadloco::dat::Circle const aNull{};
		if ( isValid(aNull))
		{
			oss << "Failure of null Circle instance test\n";
			oss << "aNull: " << aNull << '\n';
		}

		// [DoxyExample00]

		// a circle at some location
		quadloco::dat::Spot const center{ 10., 20. };
		constexpr double rho{ 2. };
		constexpr double radius{ rho };
		quadloco::dat::Circle const circle{ center, radius };

		// simple intersection test - with vertical line
		constexpr double dx{ 1. };
		quadloco::dat::Spot const spotOnLine
			{ center + quadloco::dat::Spot{ dx, 0. } };
		quadloco::pix::Gradel const gradel{ 1., 0. };
		quadloco::dat::Vec2D const lineDir
			{ quadloco::fnd::lineDirFromEdgeDir(gradel) };

		// circle intersection with vertical line
		quadloco::dat::CircleIntersector const intersector{ circle };
		std::pair<quadloco::dat::Spot, quadloco::dat::Spot> const gotPair
			{ intersector(spotOnLine, lineDir) };

		// intersection points are symmetric above/below the test point
		std::pair<quadloco::dat::Spot, quadloco::dat::Spot> const expPair
			{ quadloco::dat::Spot{ 11., 20. - std::sqrt(rho*rho-dx*dx) }
			, quadloco::dat::Spot{ 11., 20. + std::sqrt(rho*rho-dx*dx) }
			};

		// [DoxyExample00]

		bool const samePair
			{  nearlyEquals(gotPair.first, expPair.first)
			&& nearlyEquals(gotPair.second, expPair.second)
			};
		if (! samePair)
		{
			oss << "Failure of gotPair solution test\n";
			oss << "exp:"
				<< " (" << expPair.first << ") "
				<< " (" << expPair.second << ") "
				<< '\n';
			oss << "got:"
				<< " (" << gotPair.first << ") "
				<< " (" << gotPair.second << ") "
				<< '\n';
		}

		double const rejCir1
			{ rejectFromCircle(gotPair.first, circle) };
		double const rejCir2
			{ rejectFromCircle(gotPair.second, circle) };
		double const rejLine1
			{ rejectFromLine(gotPair.first, spotOnLine, lineDir) };
		double const rejLine2
			{ rejectFromLine(gotPair.second, spotOnLine, lineDir) };

		constexpr double tol{ 128. * std::numeric_limits<double>::epsilon() };
		if (! engabra::g3::nearlyEquals(rejCir1, 0., tol))
		{
			oss << "Failure of rejCir1 test\n";
			oss << "exp: " << 0. << '\n';
			oss << "got: " << rejCir1 << '\n';
		}
		if (! engabra::g3::nearlyEquals(rejCir2, 0., tol))
		{
			oss << "Failure of rejCir2 test\n";
			oss << "exp: " << 0. << '\n';
			oss << "got: " << rejCir2 << '\n';
		}
		if (! engabra::g3::nearlyEquals(rejLine1, 0.))
		{
			oss << "Failure of rejLine1 test\n";
			oss << "exp: " << 0. << '\n';
			oss << "got: " << rejLine1 << '\n';
		}
		if (! engabra::g3::nearlyEquals(rejLine2, 0.))
		{
			oss << "Failure of rejLine2 test\n";
			oss << "exp: " << 0. << '\n';
			oss << "got: " << rejLine2 << '\n';
		}

	}

	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample00]

		// a circle at some location
		quadloco::dat::Spot const center{ 10., 20. };
		constexpr double radius{ 5.25 };
		quadloco::dat::Circle const circle{ center, radius };

		// simple intersection test - with vertical line
		quadloco::dat::Spot const spotOnLine{ 12., 18. };
		quadloco::pix::Gradel const gradel{ -1.25, 1.75 };

		// circle intersection with vertical line
		quadloco::dat::CircleIntersector const intersector{ circle };
		quadloco::dat::Vec2D const lineDir
			{ quadloco::fnd::lineDirFromEdgeDir(gradel) };
		std::pair<quadloco::dat::Spot, quadloco::dat::Spot> const gotPair
			{ intersector(spotOnLine, lineDir) };

		// [DoxyExample00]

		// check if each solution is on circle
		double const rejCir1
			{ rejectFromCircle(gotPair.first, circle) };
		double const rejCir2
			{ rejectFromCircle(gotPair.second, circle) };

		// check if each solution is on line
		double const rejLine1
			{ rejectFromLine(gotPair.first, spotOnLine, lineDir) };
		double const rejLine2
			{ rejectFromLine(gotPair.second, spotOnLine, lineDir) };

		constexpr double tol{ 64. * std::numeric_limits<double>::epsilon() };
		bool const onCircle
			{  engabra::g3::nearlyEquals(rejCir1, 0., tol)
			&& engabra::g3::nearlyEquals(rejCir2, 0., tol)
			};
		bool const onLine
			{  engabra::g3::nearlyEquals(rejLine1, 0., tol)
			&& engabra::g3::nearlyEquals(rejLine2, 0., tol)
			};
		if (! onCircle)
		{
			oss << "Failure of on Circle test\n";
			oss << "rejCir1: " << rejCir1 << '\n';
			oss << "rejCir2: " << rejCir2 << '\n';
			oss << "gotPair:"
				<< ' ' << gotPair.first << ' ' << gotPair.second << '\n';
		}
		if (! onLine)
		{
			oss << "Failure of on Line test\n";
			oss << "rejLine1: " << rejLine1 << '\n';
			oss << "rejLine2: " << rejLine2 << '\n';
			oss << "gotPair:"
				<< ' ' << gotPair.first << ' ' << gotPair.second << '\n';
		}

	}

}

//! Standard test case main wrapper
int
main
	()
{
	int status{ 1 };
	std::stringstream oss;

	test0(oss);
	test1(oss);

	if (oss.str().empty()) // Only pass if no errors were encountered
	{
		status = 0;
	}
	else
	{
		// else report error messages
		std::cerr << "### FAILURE in test file: " << __FILE__ << std::endl;
		std::cerr << oss.str();
	}
	return status;
}

