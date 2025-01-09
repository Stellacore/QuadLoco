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
\brief Unit tests (and example) code for quadloco::ang::Ring
*/


#include "angRing.hpp"

#include "ang.hpp"
#include "imgSpot.hpp"
#include "prbGauss1D.hpp"

#include <Engabra>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
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
		quadloco::ang::Ring const aNull{};
		bool const expIsValid{ false };
		bool const gotIsValid{ isValid(aNull) };

		// principal angle - in exact half open interval [-pi,+pi)
		using quadloco::ang::principalAngle;
		double const expMain0{ -quadloco::ang::piOne() };
		double const gotMain1 // start of interval
			{ principalAngle(-quadloco::ang::piOne()) };
		double const gotMain2 // end wraps back to start
			{ principalAngle(quadloco::ang::piOne()) };

		// non-negative angle - in exact half open interval [0,+2pi)
		using quadloco::ang::nonNegativeAngle;
		double const expPos1{ quadloco::ang::piOne() };
		double const gotPos1{ nonNegativeAngle(-quadloco::ang::piOne()) };
		double const expPos2{ quadloco::ang::piTwo() - .5 };
		double const gotPos2{ nonNegativeAngle(-.5) };

		// check angleDelta() and size relationship
		constexpr std::size_t nBins{ 5u };
		quadloco::ang::Ring const ring(nBins);
		double const gotFullTurn{ ring.angleDelta() * (double)ring.size() };
		double const expFullTurn{ 2. * std::numbers::pi_v<double> };

		// [DoxyExample00]

		if (! (gotIsValid == expIsValid))
		{
			oss << "Failure of aNull validity test\n";
			oss << "aNull: " << aNull << '\n';
		}

		if (! quadloco::ang::nearlySameAngle(gotMain1, expMain0))
		{
			oss << "Failure of gotMain1 test\n";
			oss << "exp: " << expMain0 << '\n';
			oss << "got: " << gotMain1 << '\n';
		}
		if (! quadloco::ang::nearlySameAngle(gotMain2, expMain0))
		{
			oss << "Failure of gotMain2 test\n";
			oss << "exp: " << expMain0 << '\n';
			oss << "got: " << gotMain2 << '\n';
		}

		if (! quadloco::ang::nearlySameAngle(gotPos1, expPos1))
		{
			oss << "Failure of gotPos1 test\n";
			oss << "exp: " << expPos1 << '\n';
			oss << "got: " << gotPos1 << '\n';
		}
		if (! quadloco::ang::nearlySameAngle(gotPos2, expPos2))
		{
			oss << "Failure of gotPos2 test\n";
			oss << "exp: " << expPos2 << '\n';
			oss << "got: " << gotPos2 << '\n';
		}

		if (! engabra::g3::nearlyEquals(gotFullTurn, expFullTurn))
		{
			using engabra::g3::io::fixed;
			oss << "Failure of gotFullTurn angle test\n";
			oss << "exp: " << fixed(expFullTurn) << '\n';
			oss << "got: " << fixed(gotFullTurn) << '\n';
		}

		// constexpr std::size_t nBins{ 5u };
		// quadloco::ang::Ring const ring(nBins);
		std::size_t const ndxWrap4{ ring.indexRelativeTo(0u, -1) }; // exp 4
		std::size_t const ndxWrap0{ ring.indexRelativeTo(0u,  5) }; // exp 0
		std::size_t const ndxWrap1{ ring.indexRelativeTo(0u, -4) }; // exp 1
		if ( (! (4 == ndxWrap4))
		  || (! (0 == ndxWrap0))
		  || (! (1 == ndxWrap1))
		   )
		{
			oss << "Failure of ndxWrap test\n";
			oss << "ndxWrap4: " << ndxWrap4 << '\n';
			oss << "ndxWrap0: " << ndxWrap0 << '\n';
			oss << "ndxWrap1: " << ndxWrap1 << '\n';
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
		quadloco::ang::Ring const ring(numParts);
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
				{ quadloco::ang::principalAngle(angle) };

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
				{ quadloco::ang::atan2
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
		quadloco::ang::Ring const ring(numParts);

		double const gotAng0{ ring.angleAt(0u) };
		double const expAng0{ -quadloco::ang::piOne() };

		double const gotAng1{ ring.angleAt(1u) };
		double const expAng1{ -.5*quadloco::ang::piOne() };

		double const gotAng2{ ring.angleAt(2u) };
		double const expAng2{ 0. };

		double const gotAng3{ ring.angleAt(3u) };
		double const expAng3{  .5*quadloco::ang::piOne() };

		// [DoxyExample02]

		if (! quadloco::ang::nearlySameAngle(gotAng0, expAng0))
		{
			oss << "Failure of gotAng0 test\n";
			oss << "exp: " << expAng0 << '\n';
			oss << "got: " << gotAng0 << '\n';
		}
		if (! quadloco::ang::nearlySameAngle(gotAng1, expAng1))
		{
			oss << "Failure of gotAng1 test\n";
			oss << "exp: " << expAng1 << '\n';
			oss << "got: " << gotAng1 << '\n';
		}
		if (! quadloco::ang::nearlySameAngle(gotAng2, expAng2))
		{
			oss << "Failure of gotAng2 test\n";
			oss << "exp: " << expAng2 << '\n';
			oss << "got: " << gotAng2 << '\n';
		}
		if (! quadloco::ang::nearlySameAngle(gotAng3, expAng3))
		{
			oss << "Failure of gotAng3 test\n";
			oss << "exp: " << expAng3 << '\n';
			oss << "got: " << gotAng3 << '\n';
		}

	}

	inline
	std::vector<quadloco::img::Spot>
	spotsAbout
		( quadloco::img::Spot const & meanSpot
		, double const & sigma
		, std::size_t const & numSpots
		)
	{
		std::vector<quadloco::img::Spot> spots;
		static std::mt19937 gen{ 77588584u };
		static std::normal_distribution<double> dist0(meanSpot[0], sigma);
		static std::normal_distribution<double> dist1(meanSpot[1], sigma);
		for (std::size_t nn{0u} ; nn < numSpots ; ++nn)
		{
			quadloco::img::Spot const spot{ dist0(gen), dist1(gen) };
			spots.emplace_back(spot);
		}
		return spots;
	}

	inline
	std::vector<double>
	anglesFromSpots
		( std::vector<quadloco::img::Spot> const & spots
		)
	{
		std::vector<double> angles;
		angles.reserve(spots.size());
		for (quadloco::img::Spot const & spot : spots)
		{
			angles.emplace_back(quadloco::ang::atan2(spot[1], spot[0]));
		}
		return angles;
	}

	//! Check use as an accumulation buffer
	void
	test3
		( std::ostream & oss
		)
	{
		// [DoxyExample03]

		using namespace quadloco;

		// generate (normaly distributed) random samples about expSpot
		// and convert to angles
		img::Spot const expSpot{ -1., 2. };
		double const sigma{ magnitude(expSpot) };
		constexpr std::size_t numSpots{ 16u*1024u }; // many for smooth peak
		std::vector<img::Spot> const spots
			{ spotsAbout(expSpot, sigma, numSpots) };
		std::vector<double> const angles{ anglesFromSpots(spots) };
		double const expAngle{ ang::atan2(expSpot[1], expSpot[0]) };

		// accumulate angles into ring buffer
		std::size_t const numBins{ 32u };
		ang::Ring const ring(numBins);
		std::vector<double> binSums(numBins, 0.);
		for (double const & angle : angles)
		{
			// use buffer index manipulation functions
			std::size_t const ndxCurr{ ring.indexFor(angle) };
			std::size_t const ndxPrev{ ring.indexRelativeTo(ndxCurr, -1) };
			std::size_t const ndxNext{ ring.indexRelativeTo(ndxCurr, 1) };
			// use gaussian function to distribute in adjacent bins
			static prb::Gauss1D const gauss(0., ring.angleDelta());
			// distance into current bin
			double const offset{ angle - ring.angleAt(ndxCurr) };
			binSums[ndxPrev] += gauss(offset - ring.angleDelta());
			binSums[ndxCurr] += gauss(offset);
			binSums[ndxNext] += gauss(offset + ring.angleDelta());
		}

		// find bin with maximum accumulation value
		std::vector<double>::const_iterator const itMax
			{ std::max_element(binSums.cbegin(), binSums.cend()) };
		std::size_t const ndxMax
			{ (std::size_t)std::distance(binSums.cbegin(), itMax) };
		double const gotAngle{ ring.angleAt(ndxMax) };
		double const tol{ ring.angleDelta() };

		// [DoxyExample03]

		/*
		for (std::size_t ndx{0u} ; ndx < binSums.size() ; ++ndx)
		{
			std::cout << "binSum:"
				<< ' ' << std::setw(3u) << ndx
				<< ' ' << engabra::g3::io::fixed(binSums[ndx])
				<< ' ' << engabra::g3::io::fixed(ring.angleAt(ndx))
				<< '\n';
		}
		std::cout << "ndxMax: " << ndxMax << '\n';
		*/

		if (! ang::nearlySameAngle(gotAngle, expAngle, tol))
		{
			oss << "Failure of gotAngle buffer test\n";
			oss << "exp: " << expAngle << '\n';
			oss << "got: " << gotAngle << '\n';
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
	test3(oss);

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

