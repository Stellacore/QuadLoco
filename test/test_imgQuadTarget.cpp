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
\brief Unit tests (and example) code for quadloco::img::QuadTarget
*/


#include "imgQuadTarget.hpp"

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

		quadloco::img::QuadTarget const aNull{};

		// [DoxyExample00]

		if ( isValid(aNull))
		{
			oss << "Failure of null isValid(aNull) test\n";
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

		// NOTE: image is 2D space, but implemented with engaba::g3
		//       as hack/easy way to provide basic vector ops support.
		quadloco::img::Spot const center{ 34.5, 67.8 };

		// setup direction of axes
		constexpr double angleX{  .125 };
		constexpr double angleY{ 1.250 };
		using quadloco::img::Vector;
		Vector<double> const dirX{ std::cos(angleX), std::sin(angleX) };
		Vector<double> const dirY{ std::cos(angleY), std::sin(angleY) };

		// construct a quad
		constexpr double centerSigma{ 1./16. };
		quadloco::img::QuadTarget const imgQuad
			{ center,  dirX,  dirY, centerSigma };

		// get (image space) angle from X to Y
		double expAngleYwX{ angleY - angleX };
		double const gotAngleYwX{ imgQuad.angleSizeYwX() };

		// the (ideal) images are symmetric under half turn rotation
		quadloco::img::QuadTarget const imgQuadA{ imgQuad }; // copy ctor
		quadloco::img::QuadTarget const imgQuadB
			{ center, -dirX, -dirY, centerSigma };
		bool const expSame{ true };
		bool const gotSame{ nearlyEquals(imgQuadA, imgQuadB) };

		// [DoxyExample01]

		if (! isValid(imgQuad))
		{
			oss << "Failure of isValid(imgQuad) test\n";
		}

		if (! (gotSame == expSame))
		{
			oss << "Failure of imgQuadTarget half turn test\n";
			oss << "imgQuadA: " << imgQuadA << '\n';
			oss << "imgQuadB: " << imgQuadB << '\n';
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

