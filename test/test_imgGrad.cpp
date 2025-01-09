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
\brief Unit tests (and example) code for quadloco::img::Grad
*/


#include "imgGrad.hpp"
#include "rasgrid.hpp"
#include "rasGrid.hpp"

#include <algorithm>
#include <iostream>
#include <sstream>


namespace
{
	//! Check null/valid instances
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample00]

		// null pixel gradient element
		quadloco::img::Grad const aNull{};
		bool const expNull{ false };
		bool const gotNull{ isValid(aNull) };

		// valid pixel gradent element
		quadloco::img::Grad const aOkay{ 1.25, -2.15 };
		bool const expOkay{ true };
		bool const gotOkay{ isValid(aOkay) };

		// [DoxyExample00]

		if (! (gotOkay == expOkay))
		{
			oss << "Failure of Okay img::Grad element test(0)\n";
			oss << "exp: " << expOkay << '\n';
			oss << "got: " << gotOkay << '\n';
			oss << "aOkay: " << aOkay << '\n';
		}

		if (! (gotNull == expNull))
		{
			oss << "Failure of Null img::Grad element test(0)\n";
			oss << "exp: " << expNull << '\n';
			oss << "got: " << gotNull << '\n';
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

		// [DoxyExample01]
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

