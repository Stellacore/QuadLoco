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
\brief Unit tests (and example) code for quadloco::hough::ParmAD
*/


#include "houghParmAD.hpp"

#include <iostream>
#include <sstream>


namespace
{
	//! Examples for documentation
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample00]

		// create a null instance
		quadloco::hough::ParmAD const aNull{};
		bool const expValid{ false };
		bool const gotValid{ isValid(aNull) };

		// orthogonal line direction (e.g. lineDir is 1/4 RH turn from gradDir)
		quadloco::pix::Grad const gradDir{ 1., 0. };
		quadloco::dat::Vec2D<double> const expLineDir{ 0., 1. }; // RH 1/4 turn
		quadloco::dat::Vec2D<double> const gotLineDir
			{ quadloco::hough::ParmAD::lineDirFromEdgeDir(gradDir) };

		// [DoxyExample00]

		if (! (gotValid == expValid))
		{
			oss << "Failure of aNull validity test(0)\n";
			oss << "exp: " << expValid << '\n';
			oss << "got: " << gotValid << '\n';
		}

		if (! nearlyEquals(gotLineDir, expLineDir))
		{
			oss << "Failure of gotLineDir test(0)\n";
			oss << "exp: " << expLineDir << '\n';
			oss << "got: " << gotLineDir << '\n';
		}

	}

	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		// bounding circle
		quadloco::dat::Spot const center{ 10., 20. };
		constexpr double radius{ 10. };
		quadloco::dat::Circle const circle{ center, radius };

		// raster edge element
		using namespace quadloco::pix;
		Spot const pixLoc{ 10.f, 21.f }; // shift in col has no effect
		Grad const pixGrad{ 1.f, 0.f }; // grad in row dir
		Edgel const edgel{ pixLoc, pixGrad };

		// the line segmeng of interest is perpendicular to edge gradient
		quadloco::dat::Vec2D<double> const lineDir
			{ quadloco::hough::ParmAD::lineDirFromEdgeDir(edgel.gradient()) };

		// the Alpha,Delta parameters associate with the line segment
		static const double piHalf{ 2.*std::atan(1.) };
		static const double pi{ 2.*piHalf };
		quadloco::hough::ParmAD const expParmAD{ -piHalf, pi };
		quadloco::hough::ParmAD const gotParmAD
			{ quadloco::hough::ParmAD::from(edgel, circle) };


		// [DoxyExample01]

		if (! nearlyEquals(gotParmAD, expParmAD))
		{
			oss << "Failure of gotParmAD test(1)\n";
			oss << "exp: " << expParmAD << '\n';
			oss << "got: " << gotParmAD << '\n';
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

