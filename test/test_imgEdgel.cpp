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
\brief Unit tests (and example) code for quadloco::img::Edgel
*/


#include "QuadLoco/cast.hpp"
#include "QuadLoco/rasRowCol.hpp"
#include "QuadLoco/rasSizeHW.hpp"
#include "QuadLoco/imgEdgel.hpp"

#include <iostream>
#include <sstream>


namespace
{
	//! Examples for documentation
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample00]

		// null instance management
		quadloco::img::Edgel const aNull{};
		bool const expValid{ false };
		bool const gotValid{ isValid(aNull) };

		// [DoxyExample00]

		if (! (gotValid == expValid))
		{
			oss << "Failure of aNull validity test(0)\n";
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

		// Edge element (assume in middle of raster)
		constexpr quadloco::ras::SizeHW hwSize{ 1024u, 2048u };
		quadloco::img::Edgel const edgel
			{ quadloco::img::Spot{ 500.25f, 1000.75f } // edge location
//			, quadloco::img::Grad{ 1.75f, 2.25f }  // gradient at that loc
			, quadloco::img::Grad{ 1.00f, 0.00f }  // gradient at that loc
			};

		// with a gradient facing right/lower-right
		// expect hwSize(0u,0u) to be "behind" the gradient
		// expect hwSize(high(),wide()) to be "in front of" the gradient
		// expect location of gradient edge itself to be "in-front"
		quadloco::img::Spot const spotBack{ 0., 0. };
		quadloco::img::Spot const spotFront
			{ hwSize.high()-1u, hwSize.wide()-1u };
		bool const expBack{ false };
		bool const gotBack{ edgel.isAhead(spotBack) };
		bool const expFront{ true };
		bool const gotFront{ edgel.spotInFront(spotFront) };
		quadloco::img::Spot const spotOnEdge{ edgel.location() }; // not behind
		bool const expOnEdge{ true };
		bool const gotOnEdge{ edgel.isAhead(spotOnEdge) };

		// [DoxyExample01]

		if (! isValid(edgel))
		{
			oss << "Failure of edgel validity test(1)\n";
			oss << "edgel: " << edgel << '\n';
		}

		if (! (gotBack == expBack))
		{
			oss << "Failure of gotBack test(1)\n";
			oss << "exp: " << expBack << '\n';
			oss << "got: " << gotBack << '\n';
		}
		if (! (gotFront == expFront))
		{
			oss << "Failure of gotFront test(1)\n";
			oss << "exp: " << expFront << '\n';
			oss << "got: " << gotFront << '\n';
		}
		if (! (gotOnEdge == expOnEdge))
		{
			oss << "Failure of gotOnEdge test(1)\n";
			oss << "exp: " << expOnEdge << '\n';
			oss << "got: " << gotOnEdge << '\n';
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

