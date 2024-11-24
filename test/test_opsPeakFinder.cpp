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
\brief Unit tests (and example) code for quadloco::ops::PeakFinder
*/


#include "opsPeakFinder.hpp"

#include <iostream>
#include <set>
#include <sstream>
#include <vector>


namespace
{
	//! Test static functions (which do the peak finding)
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		// An arbitrary sequence of data with many local peaks
		std::vector<int> const values
			{ 8, 5, 5, 5, 6, 5, 7, 8, 8, 9, 9, 5, 5, 4, 6, 3, 4, 6, 7, 7
			, 7, 5, 6, 7, 8, 8, 6, 5, 7, 4, 3, 5, 4, 3, 3, 2, 1, 3, 2, 4
			};
		// Peaks are any location for which before/after are both NOT larger.
		// Grouping on lines below corresponds with content of sets to be
		// returned by PeakFinder::peakIndexGrps() function.
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

		// Find peaks in data value sequence (with wrap around)
		// (For peak w/o wrap, ignore ndx==0, ndx==size()-1u results)
		std::vector<std::vector<std::size_t> > const peakNdxGrps
			{ quadloco::ops::PeakFinder::peakIndexGroups
				(values.cbegin(), values.cend())
			};

		// [DoxyExample01]

		// raw classifications of data changes
		/*
		// quadloco::ops::PeakFinder const pf;
		std::vector<quadloco::ops::PeakFinder::NdxFlag> const ndxFlags
			{ quadloco::ops::PeakFinder::ndxFlagsFor
				(values.cbegin(), values.cend())
			};
		for (quadloco::ops::PeakFinder::Flag const & flag : flags)
		{
			std::cout << "flag: " << flag << '\n';
		}
		*/

		// Evaluate individual peaks
		std::vector<std::size_t> gotNdxs;
		std::ostringstream msg;
		for (std::vector<std::size_t> const & peakNdxGrp : peakNdxGrps)
		{
			msg << "peakNdxGrp: ";
			for (std::vector<std::size_t>::const_iterator
				iter{peakNdxGrp.cbegin()}
				; peakNdxGrp.cend() != iter ; ++iter)
			{
				gotNdxs.emplace_back(*iter);
				msg << ' ' << *iter;
			}
			msg << '\n';
		}
		constexpr bool showPeakGrp{ false };
		if (showPeakGrp)
		{
			std::cout << msg.str() << '\n';
		}

		// check if all (and only) expected peaks were found
		std::vector<std::size_t> const expNdxs
			(expNdxPeaks.cbegin(), expNdxPeaks.cend());
		// sort for intersection test below
		std::sort(gotNdxs.begin(), gotNdxs.end());

		if (! (gotNdxs.size() == expNdxs.size()))
		{
			oss << "Failure of peak gotNdxs.size() test\n";
			oss << "expNdxs.size(): " << expNdxs.size() << '\n';
			oss << "gotNdxs.size(): " << gotNdxs.size() << '\n';
		}
		else
		{
			std::vector<std::size_t> sharedNdxs;
			std::set_intersection
				( gotNdxs.cbegin(), gotNdxs.cend()
				, expNdxs.cbegin(), expNdxs.cend()
				, std::inserter(sharedNdxs, sharedNdxs.end())
				);
			if (! (sharedNdxs.size() == expNdxs.size()))
			{
				oss << "Failure of peak sharedNdx intersection test\n";
				for (std::vector<std::size_t>::const_iterator
					iter{sharedNdxs.cbegin()}
					; sharedNdxs.cend() != iter ; ++iter
					)
				{
					oss << "sharedNdx: " << *iter << '\n';
				}
			}
		}

	}

	//! Check class operations
	void
	test2
		( std::ostream & oss
		)
	{
		// [DoxyExample02]

		// An arbitrary sequence of data with many local peaks
		std::vector<int> const values
			{ 8, 5, 5, 5, 6, 5, 7, 8, 8, 9, 9, 5, 5, 4, 6, 3, 4, 6, 7, 7
			, 7, 5, 6, 7, 8, 8, 6, 5, 7, 4, 3, 5, 4, 3, 3, 2, 1, 3, 2, 4
			};
		// Peaks are any location for which before/after are both NOT larger.
		// Grouping on lines below corresponds with content of sets to be
		// returned by PeakFinder::peakIndexGrps() function.
		// for flat-top peaks, expect the middle of the peak indices
		std::vector<std::size_t> const expPeakLocs
			{  0u // 0u  // single peak at start of (wrapped) data stream
			,  4u // 4u
			,  9u // 9u, 10u  // peak with two same values
			, 14u // 14u
			, 19u // 18u, 19u, 20u // peak with three same values
			, 24u // 24u, 25u  // two-wide peak
			, 28u // 28u
			, 31u // 31u
			, 37u // 37u
			};

		// Construct peak finder (assuming data wrap around)
		// (For peak w/o wrap, ignore ndx==0, ndx==size()-1u results)
		quadloco::ops::PeakFinder const peakFinder
				(values.cbegin(), values.cend());

		// Retrieve index at middle of each peak group
		std::vector<std::size_t> const gotPeakLocs{ peakFinder.peakIndices() };

		// [DoxyExample02]

		if (! (gotPeakLocs.size() == expPeakLocs.size()))
		{
			oss << "Failure of peak (middle) finding size test\n";
			oss << "expSize: " << expPeakLocs.size() << '\n';
			oss << "gotSize: " << gotPeakLocs.size() << '\n';
		}
		else
		{
			bool const same
				{ std::equal
					( gotPeakLocs.cbegin(), gotPeakLocs.cend()
					, expPeakLocs.cbegin()
					)
				};
			if (! same)
			{
				oss << "Failure of peak (middle) content test\n";
				for (std::size_t nn{0u} ; nn < expPeakLocs.size() ; ++nn)
				{
					oss << "nn: " << nn << '\n';
					oss << "exp: " << expPeakLocs[nn] << '\n';
					oss << "got: " << gotPeakLocs[nn] << '\n';
				}
			}
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
	test2(oss);

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

