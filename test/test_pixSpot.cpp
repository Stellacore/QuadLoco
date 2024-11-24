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
\brief Unit tests (and example) code for quadloco::pix::Spot
*/


#include "pixSpot.hpp"

#include <iostream>
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

		// construct a null instance
		quadloco::pix::Spot const aNull{};
		bool const nullIsOkay{ (false == isValid(aNull)) };

		// inherits most methods from img::Vec2D<float>
		// with a few name convenience forwards
		constexpr float expRow{ 2.5f };
		constexpr float expCol{ -1.75f };
		quadloco::pix::Spot const expSpot{ expRow, expCol };
		float const gotRow{ expSpot.row() };
		float const gotCol{ expSpot.col() };
		quadloco::pix::Spot const gotSpot{ gotRow, gotCol };

		// [DoxyExample01]

		if (! nullIsOkay)
		{
			oss << "Failure of aNull test\n";
			oss << "aNull: " << aNull << '\n';
		}

		if (! nearlyEquals(gotSpot, expSpot))
		{
			oss << "Failure of gotSpot test\n";
			oss << "exp: " << expSpot << '\n';
			oss << "got: " << gotSpot << '\n';
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

