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
\brief Unit tests (and example) code for quadloco::prb::Stats
*/


#include "QuadLoco/prbStats.hpp"

#include <Engabra>

#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <vector>


namespace
{
	//! Square of value
	template <typename Type>
	inline
	Type
	sq
		( Type const & val
		)
	{
		return (val*val);
	}

	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		using namespace quadloco;

		// collection of values
		// Note that null values are ignored by running stats below
		double const nan{ std::numeric_limits<double>::quiet_NaN() };
		std::vector<double> const vals
			{  10., 11., 12., 13., 14., nan, 15., 16., 17., 18., 19. };
		// expected mean/var using "two-pass" computations
		double const expMean{ 145. / 10. };
		double const expVar{ prb::Stats<double>::variance(vals, expMean) };
		double const & expMin = vals.front();
		double const & expMax = vals.back();

		// create running statistics tracker
		prb::Stats<double> runStats;

		// consider incremental bunch of values
		runStats.consider(vals.cbegin(), vals.cend());

		// get current statistics
		double const gotMean{ runStats.mean() };
		double const gotVar{ runStats.variance() };
		double const gotMin{ runStats.min() };
		double const gotMax{ runStats.max() };

		// display information
		// std::cout << "runStats: " << runStats << '\n';

		// [DoxyExample01]

		if (! engabra::g3::nearlyEquals(gotMean, expMean))
		{
			oss << "Failure of gotMean test\n";
			oss << "exp: " << expMean << '\n';
			oss << "got: " << gotMean << '\n';
		}

		if (! engabra::g3::nearlyEquals(gotVar, expVar))
		{
			oss << "Failure of gotVar test\n";
			oss << "exp: " << expVar << '\n';
			oss << "got: " << gotVar << '\n';
		}

		if (! engabra::g3::nearlyEquals(gotMin, expMin))
		{
			oss << "Failure of gotMin test\n";
			oss << "exp: " << expMin << '\n';
			oss << "got: " << gotMin << '\n';
		}

		if (! engabra::g3::nearlyEquals(gotMax, expMax))
		{
			oss << "Failure of gotMax test\n";
			oss << "exp: " << expMax << '\n';
			oss << "got: " << gotMax << '\n';
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

