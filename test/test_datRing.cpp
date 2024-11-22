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
\brief Unit tests (and example) code for quadloco::dat::Ring
*/


#include <iostream>
#include "datRing.hpp"

#include <Engabra>

#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>


namespace
{
	//! Test basic operations
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample00]

		// construct a null instance
		quadloco::dat::Ring const aNull{};
		bool const expIsValid{ false };
		bool const gotIsValid{ isValid(aNull) };

		// principal angle - in exact half open interval [-pi,+pi)
		using quadloco::dat::principalAngle;
		double const expMain0{ -quadloco::dat::piOne() };
		double const gotMain1 // start of interval
			{ principalAngle(-quadloco::dat::piOne()) };
		double const gotMain2 // end wraps back to start
			{ principalAngle(quadloco::dat::piOne()) };

		// non-negative angle - in exact half open interval [0,+2pi)
		using quadloco::dat::nonNegativeAngle;
		double const expPos1{ quadloco::dat::piOne() };
		double const gotPos1{ nonNegativeAngle(-quadloco::dat::piOne()) };
		double const expPos2{ quadloco::dat::piTwo() - .5 };
		double const gotPos2{ nonNegativeAngle(-.5) };

		// [DoxyExample00]

		if (! (gotIsValid == expIsValid))
		{
			oss << "Failure of aNull validity test\n";
			oss << "aNull: " << aNull << '\n';
		}

		if (! quadloco::dat::nearlySameAngle(gotMain1, expMain0))
		{
			oss << "Failure of gotMain1 test\n";
			oss << "exp: " << expMain0 << '\n';
			oss << "got: " << gotMain1 << '\n';
		}
		if (! quadloco::dat::nearlySameAngle(gotMain2, expMain0))
		{
			oss << "Failure of gotMain2 test\n";
			oss << "exp: " << expMain0 << '\n';
			oss << "got: " << gotMain2 << '\n';
		}

		if (! quadloco::dat::nearlySameAngle(gotPos1, expPos1))
		{
			oss << "Failure of gotPos1 test\n";
			oss << "exp: " << expPos1 << '\n';
			oss << "got: " << gotPos1 << '\n';
		}
		if (! quadloco::dat::nearlySameAngle(gotPos2, expPos2))
		{
			oss << "Failure of gotPos2 test\n";
			oss << "exp: " << expPos2 << '\n';
			oss << "got: " << gotPos2 << '\n';
		}
	}

	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		std::size_t const numParts{ 4u };
		quadloco::dat::Ring const ring(numParts);
		constexpr double piOne{      std::numbers::pi_v<double> };
		constexpr double piTwo{ 2. * std::numbers::pi_v<double> };
		double const binDelta{ piTwo / (double)numParts };

		struct TestCase
		{
			double const theTestAngle;
			double const theMainAngle; // principal branch value
			std::size_t const theNdx;
		};
		std::vector<TestCase> gotTCs{};

		constexpr double angMin{ -piTwo };
		constexpr double angMax{  piTwo };
		constexpr double da{ piOne / 4. };
		double const dubTrials{ (angMax - angMin) / da };
		gotTCs.reserve((std::size_t)std::ceil(dubTrials));
		for (double angle{angMin} ; angle < angMax ; angle += da)
		{
			// get ring buffer bin for this angle value
			std::size_t const gotNdx{ ring.indexFor(angle) };

			double const mainAngle
				{ quadloco::dat::principalAngle(angle) };

			gotTCs.emplace_back(TestCase{ angle, mainAngle, gotNdx });
		}

		// [DoxyExample01]

		std::ostringstream msg;
		bool hitErr{ false };
		for (TestCase const & gotTC : gotTCs)
		{
			// obtained result
			double const & gotNdx = gotTC.theNdx;

			// expected computation
			double const & testAngle = gotTC.theTestAngle;
			double const & gotMainAngle = gotTC.theMainAngle;
			double const expMainAngle
				{ quadloco::dat::atan2
					(std::sin(testAngle), std::cos(testAngle))
				};
			double const distFromStart{ (expMainAngle + piOne) };
			double const dubBins{ distFromStart / binDelta };
			std::size_t const expNdx{ (std::size_t)std::floor(dubBins) };

			// test angles - need wrap around evaluation
			bool const okayAngle
				{ engabra::g3::nearlyEquals(gotMainAngle, expMainAngle) };
			bool const okayNdx
				{ (gotNdx == expNdx) };
			if (! (okayAngle && okayNdx))
			{
				hitErr = true;
			}

			if (! (okayAngle && okayNdx))
			{
				using engabra::g3::io::enote;
				using engabra::g3::io::fixed;
				msg << "testAngle: " << fixed(testAngle)
					<< ' '
					<< "expMainAngle: " << fixed(expMainAngle)
				//	<< " " << fixed(expMainAngle-piOne, 2u, 16u)
					<< ' '
					<< "gotMainAngle: " << fixed(gotMainAngle)
					<< ' '
					<< "difMainAngle: " << enote(gotMainAngle-expMainAngle, 3u)
					<< ' '
					<< "dubBins: " << fixed(dubBins)
					<< ' '
					<< "expNdx: " << fixed(expNdx)
					<< ' '
					<< "gotNdx: " << fixed(gotNdx)
					<< ' '
					<< "okayAngle: " << okayAngle
					<< ' '
					<< "okayNdx: " << okayNdx
					<< '\n';
			}
		}

		if (hitErr)
		{
			oss << "Failure of ring angle/index test\n";
			oss << msg.str();
		}
	}

	//! Check angle from index
	void
	test2
		( std::ostream & oss
		)
	{
		// [DoxyExample02]

		std::size_t const numParts{ 4u };
		quadloco::dat::Ring const ring(numParts);

		double const gotAng0{ ring.angleAt(0u) };
		double const expAng0{ -quadloco::dat::piOne() };

		double const gotAng1{ ring.angleAt(1u) };
		double const expAng1{ -.5*quadloco::dat::piOne() };

		double const gotAng2{ ring.angleAt(2u) };
		double const expAng2{ 0. };

		double const gotAng3{ ring.angleAt(3u) };
		double const expAng3{  .5*quadloco::dat::piOne() };

		// [DoxyExample02]

		if (! quadloco::dat::nearlySameAngle(gotAng0, expAng0))
		{
			oss << "Failure of gotAng0 test\n";
			oss << "exp: " << expAng0 << '\n';
			oss << "got: " << gotAng0 << '\n';
		}
		if (! quadloco::dat::nearlySameAngle(gotAng1, expAng1))
		{
			oss << "Failure of gotAng1 test\n";
			oss << "exp: " << expAng1 << '\n';
			oss << "got: " << gotAng1 << '\n';
		}
		if (! quadloco::dat::nearlySameAngle(gotAng2, expAng2))
		{
			oss << "Failure of gotAng2 test\n";
			oss << "exp: " << expAng2 << '\n';
			oss << "got: " << gotAng2 << '\n';
		}
		if (! quadloco::dat::nearlySameAngle(gotAng3, expAng3))
		{
			oss << "Failure of gotAng3 test\n";
			oss << "exp: " << expAng3 << '\n';
			oss << "got: " << gotAng3 << '\n';
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
	test2(oss);

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

