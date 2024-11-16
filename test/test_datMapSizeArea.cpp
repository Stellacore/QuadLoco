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
\brief Unit tests (and example) code for quadloco::dat::MapSizeArea
*/


#include "datMapSizeArea.hpp"

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

		quadloco::dat::SizeHW const hwSize{ 17u, 23u };
		quadloco::dat::Span const xSpan{ 17., 34. };
		quadloco::dat::Span const ySpan{ 23., 69. };

		quadloco::dat::MapSizeArea const cellMap(hwSize, xSpan, ySpan);

		quadloco::dat::Spot const gridSpotA{ 0., 0. };
		quadloco::dat::Spot const expAreaA{ 17., 23. }; // from {x,y}Span
		quadloco::dat::Spot const gotAreaA
			{ cellMap.areaSpotForGridSpot(gridSpotA) };

		quadloco::dat::Spot const gridSpotB{ 17., 23. };
		quadloco::dat::Spot const expAreaB{ 34., 69. }; // from {x,y}Span
		quadloco::dat::Spot const gotAreaB
			{ cellMap.areaSpotForGridSpot(gridSpotB) };

		quadloco::dat::Spot const areaSpotC{ .5*(17.+34.), .5*(23.+69.) };
		quadloco::dat::Spot const expGridC{ .5*(0.+17.), .5*(0.+23.) };
		quadloco::dat::Spot const gotGridC
			{ cellMap.gridSpotForAreaSpot(areaSpotC) };

		// [DoxyExample01]


		if (! nearlyEquals(gotAreaA, expAreaA))
		{
			oss << "Failure of gotAreaA test\n";
			oss << "exp: " << expAreaA << '\n';
			oss << "got: " << gotAreaA << '\n';
		}

		if (! nearlyEquals(gotAreaB, expAreaB))
		{
			oss << "Failure of gotAreaB test\n";
			oss << "exp: " << expAreaB << '\n';
			oss << "got: " << gotAreaB << '\n';
		}

		if (! nearlyEquals(gotGridC, expGridC))
		{
			oss << "Failure of gotGridC test\n";
			oss << "exp: " << expGridC << '\n';
			oss << "got: " << gotGridC << '\n';
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

