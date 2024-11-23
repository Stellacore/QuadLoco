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
\brief Unit tests (and example) code for quadloco::sig::PeakFinder
*/


#include "sigPeakFinder.hpp"

#include <iostream>
#include <sstream>
#include <vector>


namespace
{
	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		// an arbitrary sequence of data
		std::vector<int> const values
			{ 8, 5, 5, 5, 6, 5, 7, 8, 8, 9, 9, 5, 5, 4, 6, 3, 4, 6, 7, 7
			, 7, 5, 6, 7, 8, 8, 6, 5, 7, 4, 3, 5, 4, 3, 3, 2, 1, 3, 2, 4
			};
		// peaks are any location for which before/after are both NOT larger
		std::vector<std::size_t> const expNdxPeaks
			{ 0u  // single peak at start of (wrapped) data stream
			, 4u
			, 9u, 10u  // peak with two same values
			, 14u
			, 18u, 19u, 20u // peak with three same values
			, 24u, 25u  // two-wide peak
			, 28u
			, 31u
			, 37u
			};

		// quadloco::sig::PeakFinder const pf;
		std::vector<quadloco::sig::PeakFinder::Flag> const flags
			{ quadloco::sig::PeakFinder::flagsFor
				(values.cbegin(), values.cend())
			};
		/*
		for (quadloco::sig::PeakFinder::Flag const & flag : flags)
		{
			std::cout << "flag: " << flag << '\n';
		}
		*/

		// [DoxyExample01]

		std::vector<std::size_t> gotNdxPeaks;
		gotNdxPeaks.reserve(values.size());
		std::size_t const vSize{ values.size() };
		std::size_t const vFore{ 1u };
		std::size_t const vBack{ vSize - 1u };
		for (std::size_t ndxCurr{0u} ; ndxCurr < values.size() ; ++ndxCurr)
		{
			// brute force logic
			std::size_t const ndxPrev{ (ndxCurr + vBack) % vSize };
			std::size_t const ndxNext{ (ndxCurr + vFore) % vSize };
			int const & valPrev = values[ndxPrev];
			int const & valCurr = values[ndxCurr];
			int const & valNext = values[ndxNext];

			bool const prevNotLarger{ ! (valCurr < valPrev) };
			bool const nextNotLarger{ ! (valCurr < valNext) };
			bool const isPeak{ prevNotLarger && nextNotLarger };

			if (isPeak)
			{
				gotNdxPeaks.emplace_back(ndxCurr);
			}
/*
std::cout
	<< "ndxCurr,value: " << ndxCurr << ' ' << valCurr
	<< "   prevNot: " << prevNotLarger
	<< "   nextNot: " << nextNotLarger
	<< "    isPeak: " << isPeak
	<< '\n';
*/

		}

		/*
		for (std::size_t const & gotNdxPeak : gotNdxPeaks)
		{
			std::cout << "gotNdxPeak: " << gotNdxPeak << '\n';
		}
		*/

		// TODO replace this with real test code
		std::string const fname(__FILE__);
		bool const isTemplate{ (std::string::npos != fname.find("/_.cpp")) };
		if (! isTemplate)
		{
			oss << "Failure to implement real test\n";
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

