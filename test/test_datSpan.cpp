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
\brief Unit tests (and example) code for quadloco::dat::Span
*/


#include "datSpan.hpp"

#include <iostream>
#include <sstream>
#include <vector>


namespace
{
	//! Test case data for checking value relative to interval
	struct AtTest
	{
		enum InOut{ Null = 0, In = 100, Out = 999 };

		double const theValue;
		InOut const theIsIn;
		std::string const theName;
	};

	//! Enum value interpretation of boolean flag
	inline
	AtTest::InOut
	enumInOutFor
		( bool const & gotInBool
		)
	{
		AtTest::InOut anInOut{ AtTest::Null };
		if (gotInBool)
		{
			anInOut = AtTest::In;
		}
		else
		{
			anInOut = AtTest::Out;
		}
		return anInOut;
	}

	//! Examples for documentation
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		quadloco::dat::Span const nullSpan{};
		bool const nullIsOkay{ (false == isValid(nullSpan)) };

		// construct with subpixel row,col order
		double const expBeg{ 1.125 };
		double const expEnd{ 2.250 };
		double const expMag{ expEnd - expBeg };

		// eps needs to be big enough for meaningful add/sub with end/beg 
		double const eps{ expEnd * std::numeric_limits<double>::epsilon() };
		quadloco::dat::Span const origSpan{ expBeg, expEnd };

		// properties
		double const gotMag{ origSpan.magnitude() };

		// copy construction
		std::vector<quadloco::dat::Span> const copySpans
			{ origSpan, origSpan, origSpan };

		// output operations
		std::ostringstream msg;
		quadloco::dat::Span const copySpan{ copySpans.back() };
		msg << copySpan << '\n';

		// test values in relationship to being contained in span
		// example test cases: AtTest fields: (value, in/out of span, testId)
		std::vector<AtTest> const atTests
			{ { (expBeg - eps), AtTest::Out , "A"}
			, { (expBeg      ), AtTest::In  , "B"}
			, { (expEnd - eps), AtTest::In  , "C"}
			, { (expEnd      ), AtTest::Out , "D"}
			, { (expEnd + eps), AtTest::Out , "E"}
			};

		// [DoxyExample01]

		if (! nullIsOkay)
		{
			oss << "Failure of nullIsOkay test\n";
			oss << "nullSpan: " << nullSpan << '\n';
		}

		if (! (gotMag == expMag))
		{
			oss << "Failure of magnitude test\n";
			oss << "exp: " << expMag << '\n';
			oss << "got: " << gotMag << '\n';
		}

		if (msg.str().empty())
		{
			oss << "Failure of op<<() test\n";
		}

		std::ostringstream inErrs;
		for (AtTest const & atTest : atTests)
		{
			double const & value = atTest.theValue;
			AtTest::InOut const & expIn = atTest.theIsIn;
			std::string const & name = atTest.theName;
			bool const gotInBool{ origSpan.contains(value) };
			AtTest::InOut const gotIn{ enumInOutFor(gotInBool) };
			if (! (gotIn == expIn))
			{
				inErrs
					<< "atTest error: " << name
					<< " expIn: " << expIn
					<< " gotIn: " << gotIn
					<< " value: " << engabra::g3::io::fixed(value, 1u, 15u)
					<< " span: " << origSpan
					<< '\n';
			}
		}
		if (! inErrs.str().empty())
		{
			oss << "Failure of valueIns test\n";
			oss << inErrs.str();
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

