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
\brief Unit tests (and example) code for quadloco::img::Hit
*/


#include "imgHit.hpp"

#include <algorithm>
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
		// [DoxyExample00]

		using namespace quadloco;

		img::Hit const aNull{};
		if ( isValid(aNull)) // a null/default instance should *not* be valid
		{
			oss << "Failure of aNull test\n";
			oss << "aNull: " << aNull << '\n'; // op<<() overload
		}

		// [DoxyExample00]

	}

	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		using namespace quadloco;

		// use with collection
		img::Hit const expBig{ img::Spot{ 23., 25. }, .75, .125 };
		std::vector<img::Hit> hits
			{ img::Hit(img::Spot{ 13., 15. }, .25, .125) // smallest
			, expBig // largest
			, img::Hit(33u, 35u, .50, .125) // middle
			, img::Hit(img::Spot{ 43., 45. }, .50, .125) // middle
			};

		// sort by significance (largest to smallest - via reverse iterators)
		std::sort(hits.rbegin(), hits.rend());

		// largest (most significant) values should now be at front
		img::Hit const & gotBig = hits.front();
		bool const bigAtFront{ nearlyEquals(gotBig, expBig) };

		// [DoxyExample01]

		if (! bigAtFront)
		{
			oss << "Failure of gotBig at front test\n";
			oss << "exp: " << expBig << '\n';
			oss << "got: " << gotBig << '\n';
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

