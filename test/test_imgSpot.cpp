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
\brief Unit tests (and example) code for quadloco::img::Spot
*/


#include "QuadLoco/imgSpot.hpp"

#include <iostream>
#include <limits>
#include <sstream>
#include <vector>


namespace
{
	//! Examples for documentation
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		// construct a null instance
		quadloco::img::Spot const nullSpot{};
		bool const nullIsOkay{ (false == isValid(nullSpot)) };

		// construct with subpixel row,col order
		quadloco::img::Spot const origSpot{ 1.125, 2.250 };

		// copy construction
		std::vector<quadloco::img::Spot> const copySpots
			{ origSpot, origSpot, origSpot };

		// output operations
		std::ostringstream msg;
		quadloco::img::Spot const copySpot{ copySpots.back() };
		msg << copySpot << '\n';

		// order magnitude of data times machine eps
		constexpr double tol{ 32. * std::numeric_limits<double>::epsilon() };

		// approximate equivalence within tight tolerance
		bool const copySame{ nearlyEquals(origSpot, copySpot, tol/32.) };

		// basic arithmetic 
		quadloco::img::Spot const spotA{ 29., 23. };
		quadloco::img::Spot const spotB{  2.,  5. };
		quadloco::img::Spot const gotSum{ spotA + spotB };
		quadloco::img::Spot const expSum{ 29.+2., 23.+5. };
		quadloco::img::Spot const expDif{ 29.-2., 23.-5. };
		quadloco::img::Spot const gotDif{ spotA - spotB };
		quadloco::img::Spot const expMul{ 7.*2., 7.*5. };
		quadloco::img::Spot const gotMul{ 7.*spotB };

		// [DoxyExample01]

		if (! nullIsOkay)
		{
			oss << "Failure of null okay test\n";
			oss << "nullSpot: " << nullSpot << '\n';
		}

		if (msg.str().empty())
		{
			oss << "Failure of op<<() test\n";
		}

		if (! copySame)
		{
			oss << "Failure of copySame test\n";
			oss << "origSpot: " << origSpot << '\n';
			oss << "copySpot: " << copySpot << '\n';
		}

		if (! nearlyEquals(gotSum, expSum, tol))
		{
			oss << "Failure of sum test\n";
			oss << "expSum: " << expSum << '\n';
			oss << "gotSum: " << gotSum << '\n';
		}
		if (! nearlyEquals(gotDif, expDif, tol))
		{
			oss << "Failure of sum test\n";
			oss << "expDif: " << expDif << '\n';
			oss << "gotDif: " << gotDif << '\n';
		}
		if (! nearlyEquals(gotMul, expMul, tol))
		{
			oss << "Failure of sum test\n";
			oss << "expMul: " << expMul << '\n';
			oss << "gotMul: " << gotMul << '\n';
		}
	}

}

//! Check behavior of img::Spot
int
main
	()
{
	int status{ 1 };
	std::stringstream oss;

	test0(oss);

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

