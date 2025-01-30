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
\brief Unit tests (and example) code for quadloco::ang::AngleTracker
*/


#include "QuadLoco/opsAngleTracker.hpp"

#include <iostream>
#include <sstream>


namespace
{
	//! Check basics
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample00]

		quadloco::ops::AngleTracker const aNull{};
		bool const expIsValid{ false };
		bool const gotIsValid{ isValid(aNull) };

		// [DoxyExample00]

		if (! (gotIsValid == expIsValid))
		{
			oss << "Failure of aNull test\n";
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

		// construct an accumulation buffer - here with just few bins
		std::size_t const numBins{ 8u };
		quadloco::ops::AngleTracker angleTracker(numBins);

		// add a value near start of bin (near phase wrap location)
		double const expAngle{ .125 };
		angleTracker.consider(expAngle);

		// get angles 
		std::vector<double> const gotPeakAngles{ angleTracker.anglesOfPeaks() };
		std::size_t const expNumPeaks{ 1u };
		std::size_t const gotNumPeaks{ gotPeakAngles.size() };

		double const expProb{ 1. }; // since only one peak
		double const gotProb{ angleTracker.probAtAngle(expAngle) };

		// [DoxyExample01]

		if (! engabra::g3::nearlyEquals(gotProb, expProb))
		{
			oss << "Failure of gotProb test\n";
			oss << "exp: " << expProb << '\n';
			oss << "got: " << gotProb << '\n';
		}

		if (! (gotNumPeaks == expNumPeaks))
		{
			oss << "Failure of gotNumPeaks of peaks test\n";
			oss << "exp: " << expNumPeaks << '\n';
			oss << "got: " << gotNumPeaks << '\n';
		}
		else
		{
			double const & gotAngle = gotPeakAngles.front();
			double const tolAngle{ angleTracker.angRing().angleDelta() };
			if (! engabra::g3::nearlyEqualsAbs(gotAngle, expAngle, tolAngle))
			{
				oss << "Failure of gotAngle test\n";
				oss << "exp: " << expAngle << '\n';
				oss << "got: " << gotAngle << '\n';
				oss << "tol: " << tolAngle << '\n';
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

