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
\brief Unit tests (and example) code for quadloco::ops::AdderAD
*/


#include "opsAdderAD.hpp"

#include "cast.hpp"

#include <iostream>
#include <numbers>
#include <sstream>


namespace
{
	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		// useful constants
		constexpr double pi{ std::numbers::pi_v<double> };
		constexpr double piTwo{ 2. * std::numbers::pi_v<double> };

		// [DoxyExample01]

		// create adder into grid of hwSize spanning standard parameter
		// range of -pi <= alpha < pi, 0. <= delta < 2*pi
		// NOTE: Test code below assumes size AT LEAST (1x1) or larger
		quadloco::ras::SizeHW const hwSize{ 13u, 17u };
//quadloco::ras::SizeHW const hwSize{  5u,  5u };
		quadloco::ops::AdderAD adder(hwSize);

		// define a parameter near the center of the last cell in adder grid
		quadloco::ras::RowCol const expRowCol
			{ hwSize.high()-1u, hwSize.wide()-1u };
		quadloco::sig::ParmAD const parmAD
			{  piTwo*(1. - 1./(double)hwSize.high()) - pi
			,  piTwo*(1. - 1./(double)hwSize.wide())
			};

		// grid location spot associated with parmAD
		quadloco::ras::RowCol const gotRowCol
			{ adder.rasRowColForAD(parmAD) };
		// parameters associated with grid location
		quadloco::sig::ParmAD const gotParmAD
			{ adder.sigParmADFor(quadloco::cast::imgSpot(expRowCol)) };

/*
std::cout << '\n';
std::cout << "parmAD: " << parmAD << '\n';
std::cout << "adder: " << adder << '\n';
std::cout << "gotRowCol: " << gotRowCol << '\n';
std::cout << "gotParmAD: " << gotParmAD << '\n';
std::cout << '\n';
*/

		// add a gradient magnitude value into the AD grid
		constexpr float gradMag{ 1.f };
		adder.add(parmAD, gradMag);

		// [DoxyExample01]

		float const expSum{ gradMag };
		float const gotSum{ adder.grid()(hwSize.high()-1u, hwSize.wide()-1u) };
/*
std::cout << "expSum: " << expSum << '\n';
std::cout << "gotSum: " << gotSum << '\n';
std::cout << adder.grid().infoStringContents("adder", "%5.3f") << '\n';
*/

		if (! (gotSum == expSum))
		{
			oss << "Failure of gotSum test\n";
			oss << "exp: " << expSum << '\n';
			oss << "got: " << gotSum << '\n';
			oss << adder.grid().infoStringContents("adder", "%5.3f") << '\n';
		}

		if (! nearlyEquals(gotRowCol, expRowCol))
		{
			oss << "Failure of gotRowCol test\n";
			oss << "exp: " << expRowCol << '\n';
			oss << "got: " << gotRowCol << '\n';
		}

		quadloco::sig::ParmAD const & expParmAD = parmAD;
		double const tol
			{ piTwo / (double)std::min(hwSize.high(), hwSize.wide()) };
		if (! nearlyEquals(gotParmAD, expParmAD, .25))
		{
			oss << "Failure of gotParmAD test\n";
			oss << "exp: " << expParmAD << '\n';
			oss << "got: " << gotParmAD << '\n';
			oss << "tol: " << tol << '\n';
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

