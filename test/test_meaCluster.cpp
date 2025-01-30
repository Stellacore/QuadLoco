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
\brief Unit tests (and example) code for quadloco::mea::Cluster
*/


#include "QuadLoco/meaCluster.hpp"

#include <iostream>
#include <sstream>
#include <vector>


namespace
{
	//! check basic operations
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample00]

		// construct an null instance
		quadloco::mea::Cluster const aNull{};
		bool const gotIsValid{ isValid(aNull) };
		bool const expIsValid{ false };

		// [DoxyExample00]

		if (! (gotIsValid == expIsValid))
		{
			oss << "Failure of gotIsValid test\n";
			oss << "exp: " << expIsValid << '\n';
			oss << "got: " << gotIsValid << '\n';
			oss << "aNull: " << aNull << '\n';
		}
	}

	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		using namespace quadloco;

		// populate cloud with types derived from img::Vector
		img::Vector<double> const expLoc{ 3., 7. };
		constexpr double sigma{ 1. };
		std::vector<mea::Vector> const spots
			{ mea::Vector(expLoc+img::Spot{  1.,  0. }, sigma)
			, mea::Vector(expLoc+img::Spot{  1.,  1. }, sigma)
			, mea::Vector(expLoc+img::Spot{  0.,  1. }, sigma)
			, mea::Vector(expLoc+img::Spot{ -1.,  0. }, sigma)
			, mea::Vector(expLoc+img::Spot{ -1., -1. }, sigma)
			, mea::Vector(expLoc+img::Spot{  0., -1. }, sigma)
			};
		double const count{ (double)(spots.size()) };
		double const wgt{ 1./(sigma * sigma) };
		double const expCovar{ 1. / (count * wgt) };
		double const expSigma{ std::sqrt(expCovar) };
		mea::Vector const expCentroid(expLoc, expSigma);

		mea::Cluster const cluster(spots.cbegin(), spots.cend());
		mea::Vector const gotCentroid{ cluster.meaVectorCenter() };
		double const gotSigma{ cluster.centroidDeviationRMS() };

		// [DoxyExample01]

		if (! engabra::g3::nearlyEquals(gotSigma, expSigma))
		{
			using engabra::g3::io::fixed;
			oss << "Failure of gotSigma test\n";
			oss << "exp: " << fixed(expSigma) << '\n';
			oss << "got: " << fixed(gotSigma) << '\n';
		}

		if (! nearlyEquals(gotCentroid, expCentroid))
		{
			oss << "Failure of gotCentroid test\n";
			oss << "exp: " << expCentroid << '\n';
			oss << "got: " << gotCentroid << '\n';
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

