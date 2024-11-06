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
\brief Unit tests (and example) code for quadloco::obj::QuadTarget
*/


#include "objQuadTarget.hpp"

#include <iostream>
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
		quadloco::obj::QuadTarget const nullQuadTgt{};
		bool const nullIsOkay{ (false == isValid(nullQuadTgt)) };

		// construct with subpixel row,col order
		constexpr double fullEdgeMag{ .125 };
		constexpr double halfEdgeMag{ .5 * fullEdgeMag };
		quadloco::obj::QuadTarget const origQuadTgt{ fullEdgeMag };

		// copy construction
		std::vector<quadloco::obj::QuadTarget> const copyQuadTgts
			{ origQuadTgt, origQuadTgt, origQuadTgt };

		// output operations
		std::ostringstream msg;
		quadloco::obj::QuadTarget const copyQuadTgt{ copyQuadTgts.back() };
		msg << copyQuadTgt << '\n';

		// get radiometric values from inside the target
		constexpr double partEdgeMag{ .8 * halfEdgeMag };
		quadloco::dat::Spot const spotRT{  partEdgeMag,  partEdgeMag };
		quadloco::dat::Spot const spotLT{ -partEdgeMag,  partEdgeMag };
		quadloco::dat::Spot const spotLB{ -partEdgeMag, -partEdgeMag };
		quadloco::dat::Spot const spotRB{  partEdgeMag, -partEdgeMag };
		float const expValInRT{ 0.f };
		float const expValInLT{ 1.f };
		float const expValInLB{ 0.f };
		float const expValInRB{ 1.f };
		float const gotValInRT{ origQuadTgt.intensityAt(spotRT) };
		float const gotValInLT{ origQuadTgt.intensityAt(spotLT) };
		float const gotValInLB{ origQuadTgt.intensityAt(spotLB) };
		float const gotValInRB{ origQuadTgt.intensityAt(spotRB) };

		// [DoxyExample01]

		if (! nullIsOkay)
		{
			oss << "Failure of nullIsOkay test\n";
			oss << "nullQuadTgt: " << nullQuadTgt << '\n';
		}

		if (msg.str().empty())
		{
			oss << "Failure of op<<() test\n";
		}

		if (! (gotValInRT == expValInRT))
		{
			oss << "Failure of value in RT test\n";
			oss << "exp: " << expValInRT << '\n';
			oss << "got: " << gotValInRT << '\n';
		}
		if (! (gotValInLT == expValInLT))
		{
			oss << "Failure of value in LT test\n";
			oss << "exp: " << expValInLT << '\n';
			oss << "got: " << gotValInLT << '\n';
		}
		if (! (gotValInLB == expValInLB))
		{
			oss << "Failure of value in LB test\n";
			oss << "exp: " << expValInLB << '\n';
			oss << "got: " << gotValInLB << '\n';
		}
		if (! (gotValInRB == expValInRB))
		{
			oss << "Failure of value in RB test\n";
			oss << "exp: " << expValInRB << '\n';
			oss << "got: " << gotValInRB << '\n';
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

