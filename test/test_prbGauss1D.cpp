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
\brief Unit tests (and example) code for quadloco::prb::Gauss1D
*/


#include "QuadLoco/prbGauss1D.hpp"

#include <iostream>
#include <sstream>

#include <Engabra>


namespace
{
	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		// expected Gaussian probability density function 
		constexpr double mean{ 17. };
		constexpr double sigma{ 5. };
		quadloco::prb::Gauss1D const pdfNormal(mean, sigma);

		// at the mean value, the exp factor is 1 and value is leading constant
		constexpr double pi{ std::numbers::pi_v<double> };
		double const expAtMean { 1./(sigma * std::sqrt(2.*pi)) };
		double const gotAtMean{ pdfNormal(mean) };

		// and unit area under curve
		constexpr double expSum{ 1. };
		double gotSum{ 0. }; // e.g. Riemann sum as approximation for test
		constexpr double delVal{ 1./1024./1024. };
		constexpr double minVal{ mean - 7.*sigma };
		constexpr double maxVal{ mean + 7.*sigma };
		for (double atVal{minVal} ; atVal < maxVal ; atVal += delVal)
		{
			gotSum += delVal * pdfNormal(atVal);
		}

		// [DoxyExample01]

		if (! engabra::g3::nearlyEquals(gotAtMean, expAtMean))
		{
			oss << "Failure of pdf at mean test\n";
			oss << "exp: " << expAtMean << '\n';
			oss << "got: " << gotAtMean << '\n';
		}

		double const tol{ delVal };
		if (! engabra::g3::nearlyEquals(gotSum, expSum, tol))
		{
			using engabra::g3::io::fixed;
			oss << "Failure of cumulative pdf sum test\n";
			oss << "exp: " << fixed(expSum) << '\n';
			oss << "got: " << fixed(gotSum) << '\n';
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

