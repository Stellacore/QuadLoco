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
\brief Unit tests (and example) code for quadloco::dat::Area
*/


#include "datArea.hpp"

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
		// [DoxyExample01]

		// null area
		quadloco::dat::Area const nullArea{};
		bool const nullIsOkay{ (false == nullArea.isValid()) };

		//! construct from two Spans
		quadloco::dat::Span const span0{ 17., 19. };
		quadloco::dat::Span const span1{ 23., 29. };
		quadloco::dat::Area const anArea{ span0, span1 };

		// stream output
		std::ostringstream msg;
		msg << anArea << '\n';

		// the 'containing rules' are those of dat::Span independently
		quadloco::dat::Spot const spotIn{ 18., 24. };
		quadloco::dat::Spot const spotOut0{ 16., 24. };
		quadloco::dat::Spot const spotOut1{ 18., 30. };
		bool const expContainsA{ true };
		bool const gotContainsA{ anArea.contains(spotIn) };
		bool const expContainsB{ false };
		bool const gotContainsB{ anArea.contains(spotOut0) };
		bool const expContainsC{ false };
		bool const gotContainsC{ anArea.contains(spotOut1) };

		// [DoxyExample01]

		if (! nullIsOkay)
		{
			oss << "Failure of nullIsOkay test\n";
			oss << "nullArea: " << nullArea << '\n';
		}

		if ( msg.str().empty())
		{
			oss << "Failure of op<<() content test\n";
		}

		bool const okayContains
			{  (gotContainsA == expContainsA)
			&& (gotContainsB == expContainsB)
			&& (gotContainsC == expContainsC)
			};
		if (! okayContains)
		{
			oss << "Failure of contains test\n";
			oss << "expContainsA: " << expContainsA << '\n';
			oss << "expContainsB: " << expContainsB << '\n';
			oss << "expContainsC: " << expContainsC << '\n';
			oss << "gotContainsA: " << gotContainsA << '\n';
			oss << "gotContainsB: " << gotContainsB << '\n';
			oss << "gotContainsC: " << gotContainsC << '\n';
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

