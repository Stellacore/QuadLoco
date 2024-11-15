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
\brief Unit tests (and example) code for quadloco::hough::AdderAD
*/


#include "houghAdderAD.hpp"

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
		// [DoxyExample01]

		// create adder into grid of hwSize spanning standard parameter
		// range of -pi <= alpha < pi, 0. <= delta < 2*pi
		quadloco::dat::SizeHW const hwSize{ 13u, 17u };
		quadloco::hough::AdderAD adder(hwSize);

		// define a parameter near the center of the last cell in adder grid
		constexpr double pi{ std::numbers::pi_v<double> };
		constexpr double piTwo{ 2. * std::numbers::pi_v<double> };
		quadloco::hough::ParmAD const parmAD
			{ pi - 1./26.,  piTwo - 1./34. };

		constexpr float gradMag{ 1.f };
		adder.add(parmAD, gradMag);

		// [DoxyExample01]

		float const expSum{ 1.f };
		float const gotSum{ adder.grid()(hwSize.high()-1u, hwSize.wide()-1u) };

		if (! (gotSum == expSum))
		{
			oss << "Failure of gotSum test\n";
			oss << "exp: " << expSum << '\n';
			oss << "got: " << gotSum << '\n';
			oss << adder.grid().infoStringContents("adder", "%4.2f") << '\n';
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

