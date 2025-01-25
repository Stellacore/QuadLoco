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
\brief Unit tests (and example) code for quadloco::mea::Covar
*/


#include "meaCovar.hpp"

#include <iostream>
#include <sstream>


namespace
{
	//! Basic use
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample00]

		// construct an null instance
		quadloco::mea::Covar const aNull{};
		bool const gotIsValid{ isValid(aNull) };
		bool const expIsValid{ false };

		// [DoxyExample00]

		if (! (gotIsValid == expIsValid))
		{
			oss << "Failure of gotIsValid test\n";
			oss << "exp: " << expIsValid << '\n';
			oss << "got: " << gotIsValid << '\n';
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

		using namespace quadloco;

		// basic construction
		constexpr double expSigma{ 4. };
		double const expRMS{ expSigma };
		mea::Covar const easyCovar(expSigma);
		double const gotRMS{ easyCovar.deviationRMS() };
		mea::Covar const copyEasy(easyCovar);

		// general covariance matrix (must be symmetric)
		mat::Matrix expFullMat(2u, 2u);
		expFullMat(0u, 0u) = 2.;
		expFullMat(0u, 1u) = 3.;
		expFullMat(1u, 0u) = expFullMat(0u, 1u);
		expFullMat(1u, 1u) = 7.;
		mea::Covar const fullCovar(expFullMat);
		mat::Matrix const gotFullMat{ fullCovar.matrix() };

		// use in a collection
		std::vector<mea::Covar> const covars
			{ mea::Covar(1.)
			, mea::Covar(2.)
			, mea::Covar(3.)
			};

		// [DoxyExample01]

		constexpr double tol{ 4. * std::numeric_limits<double>::epsilon() };
		if (! nearlyEquals(gotFullMat, expFullMat, tol))
		{
			oss << "Failure of covar FullMat reconstruction test\n";
			oss << expFullMat.infoStringContents("exp:","%12.6f") << '\n';
			oss << gotFullMat.infoStringContents("got:","%12.6f") << '\n';
		}

		if (! nearlyEquals(copyEasy, easyCovar))
		{
			oss << "Failure of copyEasy test\n";
			oss << "exp: " << easyCovar << '\n';
			oss << "got: " << copyEasy << '\n';
		}

		if (! engabra::g3::nearlyEquals(gotRMS, expRMS))
		{
			using engabra::g3::io::fixed;
			oss << "Failure of gotRMS test\n";
			oss << "exp: " << fixed(expRMS) << '\n';
			oss << "got: " << fixed(gotRMS) << '\n';
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

