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
\brief Unit tests (and example) code for quadloco::sys::Timer
*/

#include <iostream> // TODO

#include "sysTimer.hpp"

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

		quadloco::sys::Timer timer{ "Name of Something" };
		// ... (something being timed)
		timer.stop();

		// get elapsed time (in [sec])
		double const gotSec{ timer.elapsed() };

		// reuse timer instance to time something else
		timer.restart("Something Else");
		// ...
		timer.stop();

		// report time to stream
		std::ostringstream strm;
		// generic info
		strm << "lineA: " << timer << '\n';
		// or display with explicitly specified number of digits
		strm << timer.infoString("lineB:", 9u) << '\n';

		// [DoxyExample01]

		if (! (0. < gotSec))
		{
			oss << "Failure of non-zero time test\n";
			oss << "exp: greater than zero\n";
			oss << "got: " << gotSec << '\n';
		}

		std::ostringstream msg;
		msg << std::fixed << std::setprecision(9u) << timer.elapsed();
		std::string const expStr{ msg.str() };
		std::string const gotStr{ strm.str() };
		if (std::string::npos == gotStr.find(expStr))
		{
			oss << "Failure of timer output string test\n";
			oss << "exp:\n" << expStr << '\n';
			oss << "got:\n" << gotStr << '\n';
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

