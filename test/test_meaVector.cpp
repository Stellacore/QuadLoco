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
\brief Unit tests (and example) code for quadloco::mea::Vector
*/


#include "meaVector.hpp"


#include <iostream>
#include <sstream>


namespace
{
	//! Examples of basic 2D measurement
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample00]

		using namespace quadloco;

		// a default null instance
		mea::Vector const aNull{};
		bool const gotIsValid{ isValid(aNull) };
		bool const expIsValid{ false };

		// a basic measurement - with uniform covariance
		img::Spot const expLoc{ -5., 3. };
		double const expSigma{ 2. };
		mea::Vector const meaPntA(expLoc, expSigma);

		// location (cast to spot) and standard deviation
		img::Spot const gotLoc{ meaPntA.location() };
		double const gotSigma{ meaPntA.deviationRMS() };


		// [DoxyExample00]

		if (! (gotIsValid == expIsValid))
		{
			oss << "Failure of gotIsValid null test0\n";
			oss << "exp: " << expIsValid << '\n';
			oss << "got: " << gotIsValid << '\n';
			oss << "aNull: " << aNull << '\n';
		}

		if (! nearlyEquals(gotLoc, expLoc))
		{
			oss << "Failure of gotLoc test0\n";
			oss << "exp: " << expLoc << '\n';
			oss << "got: " << gotLoc << '\n';
		}
		if (! engabra::g3::nearlyEquals(gotSigma, expSigma))
		{
			using engabra::g3::io::fixed;
			oss << "Failure of gotSigma test0\n";
			oss << "exp: " << fixed(expSigma) << '\n';
			oss << "got: " << fixed(gotSigma) << '\n';
		}
	}

	//! Examples of basic 2D measurement
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		using namespace quadloco;

		// a basic measurement - with general covariance
		img::Spot const expLoc{ -5., 7. };
		// define uncertainty associated with this locaiton
		double const expSigmaMin{ 2. };
		double const expSigmaMax{ 3. * expSigmaMin };
		double const expCovarMin{ expSigmaMin * expSigmaMin };
		double const expCovarMax{ expSigmaMax * expSigmaMax };
		mat::Matrix const covarMat
			{ mat::diagonal({ expCovarMin, expCovarMax }) };
		img::Vector<double> const expSemiAxisMin{ expSigmaMin, 0. };
		img::Vector<double> const expSemiAxisMax{ 0., expSigmaMax };

		// a basic measurement with full covariance (here a diagonal one)
		mea::Vector const meaPnt(expLoc, covarMat);

		// location and standard uncertainty as a ellipse
		mea::Covar const & covar = meaPnt.covar();
		img::Vector<double> const gotSemiAxisMin{ covar.semiAxisMin() };
		img::Vector<double> const gotSemiAxisMax{ covar.semiAxisMax() };

		double const gotSigma{ covar.deviationRMS() };
		double const expSigma
			{ std::hypot(expSigmaMin, expSigmaMax) / std::sqrt(2.) };

		// [DoxyExample01]

		constexpr double tol{ 4. * std::numeric_limits<double>::epsilon() };
		if (! engabra::g3::nearlyEquals(gotSigma, expSigma, tol))
		{
			using namespace engabra::g3::io;
			oss << "Failure of gotSigma test1\n";
			oss << "exp: " << fixed(expSigma) << '\n';
			oss << "got: " << fixed(gotSigma) << '\n';
		}
		if (! nearlyEquals(gotSemiAxisMin, expSemiAxisMin, tol))
		{
			oss << "Failure of gotSemiAxisMin test1\n";
			oss << "exp: " << expSemiAxisMin << '\n';
			oss << "got: " << gotSemiAxisMin << '\n';
		}
		if (! nearlyEquals(gotSemiAxisMax, expSemiAxisMax, tol))
		{
			oss << "Failure of gotSemiAxisMax test1\n";
			oss << "exp: " << expSemiAxisMax << '\n';
			oss << "got: " << gotSemiAxisMax << '\n';
			oss << "meaPnt: " << meaPnt << '\n';
		}
	}

	/*
	//! Examples for documentation
	void
	test2
		( std::ostream & oss
		)
	{
		// [DoxyExample02]

		// [DoxyExample02]

	}
	*/

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
//	test2(oss);

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

