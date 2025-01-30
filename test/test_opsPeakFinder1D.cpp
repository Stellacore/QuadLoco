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
\brief Unit tests (and example) code for quadloco::ops::PeakFinder1D
*/


#include "QuadLoco/opsPeakFinder1D.hpp"

#include <iostream>
#include <set>
#include <sstream>
#include <vector>


namespace
{
	inline
	std::string
	infoString
		( std::vector< std::vector<std::size_t> > const & peakGrps
		)
	{
		std::ostringstream oss;
		static std::string const pad("...");
		oss << pad
			<< "Group Size: " << peakGrps.size();
		for (std::size_t nGrp{0u} ; nGrp < peakGrps.size() ; ++nGrp)
		{
			std::vector<std::size_t> const & peakGrp = peakGrps[nGrp];
			oss << '\n';
			oss << pad
				<< "PeakSize[" << nGrp << "]:"
				<< ' ' << peakGrp.size()
				<< ' ' << "peakNdxs:";
			for (std::size_t const & peakNdx : peakGrp)
			{
				oss << ' ' << std::setw(2u) << peakNdx;
			}
		}
		return oss.str();
	}

	//! Test if {got,exp}PeakLocs are the same
	inline
	void
	checkPeaks
		( std::ostream & oss
		, std::vector< std::vector<std::size_t> > const & gotPeakGrps
		, std::vector< std::vector<std::size_t> > const & expPeakGrps
		, std::string const & tname
		)
	{
		bool hitErr{ false };
		if (! (gotPeakGrps.size() == expPeakGrps.size()))
		{
			oss << "Failure of peak finding size test:\n"
				<< tname << '\n';
			hitErr = true;
		}
		else
		{
			std::size_t const numGrps{ expPeakGrps.size() };
			for (std::size_t nGrp{0u} ; nGrp < numGrps ; ++nGrp)
			{
				std::vector<std::size_t> const & expNdxs = expPeakGrps[nGrp];
				std::vector<std::size_t> const & gotNdxs = gotPeakGrps[nGrp];
				if (! (gotNdxs.size() == expNdxs.size()))
				{
					oss << "Failure of peak Ndxs size test:\n"
						<< tname << '\n';
					oss << "nGrp: " << nGrp << "exp,got:sizes:"
						<< ' ' << expNdxs.size()
						<< ' ' << gotNdxs.size()
						<< '\n';
					hitErr = true;
				}
				else
				{
					bool const same
						{ std::equal
							( gotNdxs.cbegin(), gotNdxs.cend()
							, expNdxs.cbegin()
							)
						};
					if (! same)
					{
						oss << "Failure of in-peak Ndx test:\n"
							<< tname << '\n';
						hitErr = true;
					}
				}
			}
		}

		if (hitErr)
		{
			oss << "expPeakGrps:\n" << infoString(expPeakGrps) << '\n';
			oss << "gotPeakGrps:\n" << infoString(gotPeakGrps) << '\n';
			oss << '\n';
		}

	}

	using CirNdxs = std::vector< std::vector<std::size_t> >;
	using LinNdxs = std::vector< std::vector<std::size_t> >;

	struct Result
	{
		std::string const theName{};
		LinNdxs const theGotLinNdxs;
		CirNdxs const theGotCirNdxs;

	}; // Result

	struct TestCase
	{
		std::string const theName{};
		std::vector<int> const theData{};
		LinNdxs const theExpLinNdxs{};
		CirNdxs const theExpCirNdxs{};

		inline
		std::vector< std::vector<std::size_t> >
		gotLinear
			() const
		{
			using quadloco::ops::PeakFinder1D;
			return PeakFinder1D::peakIndexGroups
				(theData.cbegin(), theData.cend(), PeakFinder1D::Linear);
		}

		inline
		std::vector< std::vector<std::size_t> >
		gotCircular
			() const
		{
			using quadloco::ops::PeakFinder1D;
			return PeakFinder1D::peakIndexGroups
				(theData.cbegin(), theData.cend(), PeakFinder1D::Circle);
		}

		inline
		Result
		run
			() const
		{
			return Result{ theName, gotLinear(), gotCircular() };
		}

	}; // TestCase

} // [anon]

namespace
{
	//! Test general cases including end conditions with Circular/Linear
	void
	test0
		( std::ostream & oss
		)
	{
		std::vector<TestCase> const testCases
			{ TestCase
				{ ">> No Data"
				, {}
				, LinNdxs{ }
				, CirNdxs{ }
				}
			, TestCase
				{ ">> Start Peak"
				, { 1, 0, 0, 0, 0 }
				, LinNdxs{ { 0u } }
				, CirNdxs{ { 0u } }
				}
			, TestCase
				{ ">> End Peak"
				, { 0, 0, 0, 0, 1 }
				, LinNdxs{ { 4u } }
				, CirNdxs{ { 4u } }
				}
			, TestCase
				{ ">> All Neg"
				, { -1, -1, -1, -1, -1 }
				, LinNdxs{ }
				, CirNdxs{ }
				}
			, TestCase
				{ ">> All Zeros"
				, { 0, 0, 0, 0, 0 }
				, LinNdxs{ }
				, CirNdxs{ }
				}
			, TestCase
				{ ">> All Pos"
				, { 1, 1, 1, 1, 1 }
				, LinNdxs{ { 0u, 1u, 2u, 3u, 4u } } // one big peak (0 b4/after)
				, CirNdxs{ }  // all constant, no peaks
				}
			, TestCase
				{ ">> Middle Peak"
				, { 0, 1, 2, 0, 0 }
				, LinNdxs{ { 2u } }
				, CirNdxs{ { 2u } }
				}
			, TestCase
				{ ">> Wide Beg"
				, { 1, 1, 0, 0, 0 }
				, LinNdxs{ { 0u, 1u } }
				, CirNdxs{ { 0u, 1u } }
				}
			, TestCase
				{ ">> Wide Peak"
				, { 0, 1, 1, 0, 0 }
				, LinNdxs{ { 1u, 2u } }
				, CirNdxs{ { 1u, 2u } }
				}
			, TestCase
				{ ">> Wide End"
				, { 0, 0, 0, 1, 1 }
				, LinNdxs{ { 3u, 4u } }
				, CirNdxs{ { 3u, 4u } }
				}
			, TestCase
				{ ">> End Peak"
				, { 10, 11, 12, 13, 20 }
				, LinNdxs{ { 4u } }
				, CirNdxs{ { 4u } }
				}
			, TestCase
				{ ">> Wrap Peak"
				, { 1, 0, 0, 0, 1 }
				, LinNdxs{ { 0u }, { 4u } }
				, CirNdxs{ { 4u, 0u } }
				}
			};

		for (TestCase const & testCase : testCases)
		{
			Result const result{ testCase.run() };
			checkPeaks
				( oss
				, result.theGotLinNdxs
				, testCase.theExpLinNdxs
				, result.theName + "-Linear"
				);
			checkPeaks
				( oss
				, result.theGotCirNdxs
				, testCase.theExpCirNdxs
				, result.theName + "-Circular"
				);
		}

		// [DoxyExample00]
		// [DoxyExample00]
	}

#if 0
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
		// returned by PeakFinder1D::peakIndexGrps() function.
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
			{ quadloco::ops::PeakFinder1D::peakIndexGroups
				(values.cbegin(), values.cend())
			};

		// [DoxyExample01]

		// raw classifications of data changes
		/*
		// quadloco::ops::PeakFinder1D const pf;
		std::vector<quadloco::ops::PeakFinder1D::NdxChange> const ndxChanges
			{ quadloco::ops::PeakFinder1D::ndxChangesFor
				(values.cbegin(), values.cend())
			};
		for (quadloco::ops::PeakFinder1D::Change const & flag : flags)
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
		// (Same data as test1() but rotated one index to put peak at end)
		std::vector<int> const values
			{ 5, 5, 5, 6, 5, 7, 8, 8, 9, 9, 5, 5, 4, 6, 3, 4, 6, 7, 7, 7
			, 5, 6, 7, 8, 8, 6, 5, 7, 4, 3, 5, 4, 3, 3, 2, 1, 3, 2, 4, 8
			};
		// Peaks are any location for which before/after are both NOT larger.
		// Grouping on lines below corresponds with content of sets to be
		// returned by PeakFinder1D::peakIndexGrps() function.
		// for flat-top peaks, expect the middle of the peak indices
		std::vector<std::size_t> const expPeakLocs
			{  3u
			,  8u
			, 13u
			, 18u
			, 23u
			, 27u
			, 30u
			, 36u
			, 39u
			};

		// Construct peak finder (assuming data wrap around)
		// (For peak w/o wrap, ignore ndx==0, ndx==size()-1u results)
		quadloco::ops::PeakFinder1D const peakFinder
				(values.cbegin(), values.cend());

		// Retrieve index at middle of each peak group
		std::vector<std::size_t> const gotPeakLocs{ peakFinder.peakIndices() };

		// [DoxyExample02]

		// test detection of peaks in order coded above
		checkPeaks(oss, gotPeakLocs, expPeakLocs, "test2-asCoded");

		// check (all) cyclic permutations of the data
		std::size_t const numVals{ values.size() };
		std::vector<int> rotValues{ values };
		std::vector<std::size_t> rotPeakLocs{ expPeakLocs };
		for (std::size_t nn{0u} ; nn < numVals ; ++nn)
		{
			std::rotate
				( rotValues.begin(), rotValues.begin() + 1u
				, rotValues.end()
				);
			std::rotate
				( rotPeakLocs.begin(), rotPeakLocs.begin() + 1u
				, rotPeakLocs.end()
				);

			std::ostringstream tmsg;
			tmsg << "test2-rotate-nn=" << nn;

			checkPeaks(oss, gotPeakLocs, expPeakLocs, tmsg.str());
		}


	}

	//! check example case (broken at commit 46f3052de
	void
	test3
		( std::ostream & oss
		)
	{
		std::vector<double> const values
			{ 1.959706  // 0: Peak (end is lower)
			, 0.147268
			, 0.020545
			, 0.037892  // 3: Peak
			, 0.003328
			, 0.001077
			, 0.000112
			, 0.000002
			, 0.000000
			, 0.000000
			, 0.000000
			, 0.000000
			, 0.029304
			, 0.083700
			, 1.667888
			, 4.393755  // 15: Peak
			, 1.624745
			, 0.235455
			, 0.000000
			, 0.000000
			, 0.000000
			, 0.000000
			, 1.913586
			, 6.749526  // 23: Peak
			, 4.190713
			, 0.392528
			, 0.000000
			, 0.000000
			, 0.000000
			, 0.032921
			, 0.120138
			, 1.083869
			};
		std::vector<std::size_t> const expPeakLocs
			{ 0u, 3u, 15u, 23u };

		// Construct peak finder (assuming data wrap around)
		// (For peak w/o wrap, ignore ndx==0, ndx==size()-1u results)
		quadloco::ops::PeakFinder1D const peakFinder
				(values.cbegin(), values.cend());

		// Retrieve index at middle of each peak group
		std::vector<std::size_t> const gotPeakLocs{ peakFinder.peakIndices() };

		// test detection of peaks in order coded above
		checkPeaks(oss, gotPeakLocs, expPeakLocs, "test3-realCase");

	}
#endif

}

//! Standard test case main wrapper
int
main
	()
{
	int status{ 1 };
	std::stringstream oss;

	test0(oss);
//	test1(oss);
//	test2(oss);
//	test3(oss);

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

