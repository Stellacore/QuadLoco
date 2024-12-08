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
\brief Unit tests (and example) code for quadloco::sig::EdgeLine
*/


#include "sigEdgeLine.hpp"

#include "imgGrad.hpp"

#include <algorithm>
#include <iostream>
#include <set>
#include <sstream>
#include <utility>


namespace
{
	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		using namespace quadloco;

		// create a null instance
		sig::EdgeLine const aNull{};
		bool const expIsValid{ false };
		bool const gotIsValid{ isValid(aNull) };

		// basic construction

		// start of Edge line
		img::Spot const expStartSpot{ .5, .75 };
		//
		// edge ray (e.g. computed from image edgels)
		// - has start on the EdgeLine 
		// - has direction of raster edge gradient
		img::Spot const pntOnLine{ 1.5, 3. };
		img::Vector<double> const edgeDir
			{ direction(img::Vector<double>{ -1., .5 }) };
		img::Ray const expEdgeRay{ pntOnLine, edgeDir };
		//
		sig::EdgeLine const edgeLine
			{ sig::EdgeLine::from(expStartSpot, expEdgeRay) };

		// Angle of line from line start
		double const gotLineAngle{ edgeLine.angleOfLine() };
		img::Vector<double> const lineDelta{ pntOnLine - expStartSpot };
		double const expLineAngle{ ang::atan2(lineDelta[1], lineDelta[0]) };

		// Spot where edge line crosses unit circle (centered on start)
		img::Spot const gotUnitSpot{ edgeLine.spotOnUnitCircle() };
		img::Spot const expUnitSpot
			{ std::cos(expLineAngle), std::sin(expLineAngle) };

		// Turning moment of edge ray relative to line direction
		double const gotMoment{ edgeLine.turnMoment() };
		double const expMoment{ outer(expUnitSpot, edgeDir) };

		// [DoxyExample01]

		if (! (gotIsValid == expIsValid))
		{
			oss << "Failure of aNull validity test\n";
			oss << "aNull: " << aNull << '\n';
		}

		if (! engabra::g3::nearlyEquals(gotLineAngle, expLineAngle))
		{
			oss << "Failure of gotLineAngle test\n";
			oss << "exp: " << expLineAngle << '\n';
			oss << "got: " << gotLineAngle << '\n';
		}

		if (! nearlyEquals(gotUnitSpot, expUnitSpot))
		{
			oss << "Failure of gotUnitSpot test\n";
			oss << "exp: " << expUnitSpot << '\n';
			oss << "got: " << gotUnitSpot << '\n';
		}

		constexpr double tol{ 4.* std::numeric_limits<double>::epsilon() };
		if (! engabra::g3::nearlyEquals(gotMoment, expMoment, tol))
		{
			oss << "Failure of gotMoment test\n";
			oss << "exp: " << expMoment << '\n';
			oss << "got: " << gotMoment << '\n';
		}

	}

	//! Example edge rays likely to be encountered with a quadrant target
	inline
	std::vector<quadloco::img::Ray>
	simEdgeRays
		()
	{
		using namespace quadloco::img;
		return std::vector<Ray>
			{	// Radial Edges
			  Ray{ Spot{  1.,  0. }, Grad{  0.,  1. } }  // 0
			, Ray{ Spot{  0.,  1. }, Grad{ -1.,  0. } }  // 1
			, Ray{ Spot{ -1.,  0. }, Grad{  0., -1. } }  // 2
			, Ray{ Spot{  0., -1. }, Grad{  1.,  0. } }  // 3
				// Triangle corners
			, Ray{ Spot{  1.,  1. }, Grad{  1.,  1. } }  // 4
			, Ray{ Spot{ -1., -1. }, Grad{ -1., -1. } }  // 5
				// Target edges
			, Ray{ Spot{  2.,  0. }, Grad{ -1.,  0. } }  // 6
			, Ray{ Spot{  0.,  2. }, Grad{  0., -1. } }  // 7
			, Ray{ Spot{ -2.,  0. }, Grad{  1.,  0. } }  // 8
			, Ray{ Spot{  0., -2. }, Grad{  0.,  1. } }  // 9
			};
	}

	using NdxPair = std::pair<std::size_t, std::size_t>;

	inline
	std::string
	infoStringFor
		( std::set<NdxPair> const & ndxPairs
		, std::string const & name
		)
	{
		std::ostringstream oss;
		oss << name;
		for (NdxPair const & ndxPair : ndxPairs)
		{
			oss
				<< '\n'
				<< "   "
				<< std::setw(3u) << ndxPair.first
				<< ", "
				<< std::setw(3u) << ndxPair.second
				;
		}
		return oss.str();
	}

	//! Check opposing edge line detection
	void
	test2
		( std::ostream & oss
		)
	{
		using namespace quadloco;

		std::vector<img::Spot> const startSpots
			{	// exact solution
			  img::Spot{  .0,  .0 }
			 	// Diagonal in background toward triangle edge
			, img::Spot{  .1,  .1 }
			, img::Spot{  .2,  .2 }
			, img::Spot{  .3,  .3 }
		//	, img::Spot{  .4,  .4 }
		//	, img::Spot{  .5,  .5 }
				// Diagonal in foreground away from triangle edge
			, img::Spot{ -.1, -.1 }
			, img::Spot{ -.2, -.2 }
			, img::Spot{ -.3, -.3 }
		//	, img::Spot{ -.4, -.4 }
		//	, img::Spot{ -.5, -.5 }
			};

		std::size_t const numSpots{ startSpots.size() };
		for (std::size_t nSpot{0u} ; nSpot < numSpots ; ++nSpot)
		{
			// [DoxyExample02]

			using namespace quadloco::img;

			// construct a collection of edge lines
			std::vector<img::Ray> const edgeRays{ simEdgeRays() };
			// std::cout << edgeRays << '\n';

			// candidate test spot from which to evaluate EdgeLine oppositions
			img::Spot const & startSpot = startSpots[nSpot]; // test case

			// use available edge rays to generate edge lines from this spot
			std::vector<sig::EdgeLine> edgeLines;
			edgeLines.reserve(edgeRays.size());
			for (img::Ray const & edgeRay : edgeRays)
			{
				sig::EdgeLine const edgeLine
					{ sig::EdgeLine::from(startSpot, edgeRay) };
				edgeLines.emplace_back(edgeLine);
			}

			// Find best opposing edge line for each edge line
			std::size_t const numLines{ edgeLines.size() };
			std::set<NdxPair> gotNdxPairs;
			for (std::size_t ndx{0u} ; ndx < numLines ; ++ndx)
			{
				sig::EdgeLine const & edgeLine = edgeLines[ndx];
				constexpr double angSigma{ 1./2. };
				sig::NdxWgt const nw
					{ edgeLine.opposingNdxWgt(edgeLines, ndx, angSigma) };
				// returned NdxWgt is invalid if no strong candidates
				if (isValid(nw))
				{
					gotNdxPairs.insert(NdxPair{ ndx, nw.item() });
				}
			}

			// [DoxyExample02]

			// expect opposite radial edges (ref simEdgeRays() above)
			std::set<NdxPair> const expNdxPairs
				{ { 0u, 2u }
				, { 1u, 3u }
				, { 2u, 0u }
				, { 3u, 1u }
				};

			/*
			if (! (gotNdxPairs.size() == expNdxPairs.size()))
			{
				oss << "Failure of gotNdxPairs size test\n";
				oss << "exp: " << expNdxPairs.size() << '\n';
				oss << "got: " << gotNdxPairs.size() << '\n';
			}
			*/

			std::set<NdxPair> difNdxPairs;
			std::set_difference
				( gotNdxPairs.cbegin(), gotNdxPairs.cend()
				, expNdxPairs.cbegin(), expNdxPairs.cend()
				, std::inserter(difNdxPairs, difNdxPairs.end())
				);
			if (! difNdxPairs.empty())
			{
				oss << "Failure of gotNdxPairs difference test\n";
				oss << "startSpot: " << startSpot << '\n';
				oss << infoStringFor(expNdxPairs, "exp") << '\n';
				oss << infoStringFor(gotNdxPairs, "got") << '\n';
				oss << infoStringFor(difNdxPairs, "dif") << '\n';
			}
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

//	test0(oss);
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

