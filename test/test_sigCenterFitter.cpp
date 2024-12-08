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
\brief Unit tests (and example) code for quadloco::sig::CenterFitter
*/


#include "sigCenterFitter.hpp"

#include "imgGrad.hpp"

#include <iostream>
#include <sstream>


namespace
{
	//! Basic construction and fuctions
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample00]

		using namespace quadloco;

		sig::CenterFitter const aNull{};
		bool const expIsValid{ false };
		bool const gotIsValid{ isValid(aNull) };

		// [DoxyExample00]

		if (! (gotIsValid == expIsValid))
		{
			oss << "Failure of aNull test\n";
			oss << "aNull:\n" << aNull << '\n';
		}

	}

	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		using namespace quadloco::img;

		// Construct three (perfect) rays intersecting at expCenter
		// NOTE: these are edge rays, and the EdgeLines are perpenduclar
		//       to these.  The desired solution is where the EdgeLine
		//       intersect.
		std::vector<Ray> const edgeRays
			{ Ray{ Spot{  5.,  4. }, Grad{ -1.,  1. } }
			, Ray{ Spot{  2.,  4. }, Grad{ -2., -1. } }
			, Ray{ Spot{  1.,  1. }, Grad{  1., -2. } }
			};
		Spot const expCenter{ 3., 2. };

		// use CenterFitter to find center (add all rays with same weight
		quadloco::sig::CenterFitter fitter{};
		constexpr double wgt{ .75 };
		fitter.addRay(edgeRays[0], wgt);
		fitter.addRay(edgeRays[1], wgt);
		fitter.addRay(edgeRays[2], wgt);

		// fetch solution and weight
		quadloco::sig::SpotSigma const solnSpotSigma
			{ fitter.solutionSpotSigma() };
		// check if solution is valid
		bool const gotValidSoln{ isValid(solnSpotSigma) };
		// if solution is not valid, both these are null
		quadloco::img::Spot const & gotCenter = solnSpotSigma.spot();
		double const & gotWgt = solnSpotSigma.sigma();

		// [DoxyExample01]

		if (! gotValidSoln)
		{
			oss << "Failure of gotValidSolution test\n";
			oss << "solnSpotSigma: " << solnSpotSigma << '\n';
			oss << "fitter\n" << fitter << '\n';
		}

		constexpr double tol{ 4. * std::numeric_limits<double>::epsilon() };
		if (! nearlyEquals(gotCenter, expCenter, tol))
		{
			oss << "Failure of gotCenter test\n";
			oss << "exp: " << expCenter << '\n';
			oss << "got: " << gotCenter << '\n';
			oss << "fitter\n" << fitter << '\n';
		}

	}

	//! Evaluate weight (covariance) estimation
	void
	test2
		( std::ostream & oss
		)
	{
		using namespace quadloco::img;

		// NOTE: these are edge rays, and the EdgeLines are perpenduclar
		//       to these.  The desired solution is where the EdgeLine
		//       intersect.
		std::vector<Ray> const edgeRays
			{ Ray{ Spot{  0.,  1. }, Grad{  1.,  0. } }
			, Ray{ Spot{  0., -1. }, Grad{  1.,  0. } }
			, Ray{ Spot{  1.,  0. }, Grad{  0.,  1. } }
			, Ray{ Spot{ -1.,  0. }, Grad{  0.,  1. } }
			};
		Spot const expCenter{ 3., 2. };

		// use CenterFitter to find center (add all rays with same weight
		quadloco::sig::CenterFitter fitter{};
		constexpr double wgt{ 1. };
		fitter.addRay(edgeRays[0], wgt);
		fitter.addRay(edgeRays[1], wgt);
		fitter.addRay(edgeRays[2], wgt);
		fitter.addRay(edgeRays[3], wgt);

		// each input ray is 1 pixel error from solution
		double const expRmse{ 1. };

		// fetch solution and weight
		quadloco::sig::SpotSigma const solnSpotSigma
			{ fitter.solutionSpotSigma() };
		double const & gotRmse = solnSpotSigma.sigma();

		if (! engabra::g3::nearlyEquals(gotRmse, expRmse))
		{
			oss << "Failure of gotRmse test\n";
			oss << "exp: " << expRmse << '\n';
			oss << "got: " << gotRmse << '\n';
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

