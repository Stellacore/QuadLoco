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
\brief Unit tests (and example) code for quadloco::dat::SizeHW
*/


#include "datSizeHW.hpp"

#include "QuadLoco"

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

		// value construction
		std::size_t const expHigh{ 23u };
		std::size_t const expWide{ 27u };
		quadloco::dat::SizeHW const hwOrig{ expHigh, expWide };
		std::size_t const expSize{ expHigh * expWide };

		// copy construction
		quadloco::dat::SizeHW const hwCopy(hwOrig);
		bool const copyIsSame{ hwCopy == hwOrig };

		// attributes
		std::size_t const gotHigh{ hwOrig.high() };
		std::size_t const gotWide{ hwOrig.wide() };
		std::size_t const gotSize{ hwOrig.size() };

		// [DoxyExample01]

		if (! hwOrig.isValid())
		{
			oss << "Failure of validity test\n";
			oss << "hwOrig: " << hwOrig << '\n';
		}

		if (! copyIsSame)
		{
			oss << "Failure of copy test\n";
			oss << "hwOrig: " << hwOrig << '\n';
			oss << "hwCopy: " << hwCopy << '\n';
		}

		bool const okaySizes
			{  (gotHigh == expHigh)
			&& (gotWide == expWide)
			&& (gotSize == expSize)
			};
		if (! okaySizes)
		{
			oss << "Failure of sizes test\n";
			oss << "expHigh: " << expHigh << '\n';
			oss << "gotHigh: " << gotHigh << '\n';
			oss << "expWide: " << expWide << '\n';
			oss << "gotWide: " << gotWide << '\n';
			oss << "expSize: " << expSize << '\n';
			oss << "gotSize: " << gotSize << '\n';
		}
	}

}

//! Check behavior of NS
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

