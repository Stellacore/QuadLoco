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
\brief Unit tests (and example) code for quadloco::cast
*/


#include "cast.hpp"

#include <iostream>
#include <sstream>


namespace
{
	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		using namespace quadloco;

		// 2D data types
		dat::RowCol const srcRowCol
			{ 7u, 3u };
		dat::Spot const srcSpot
			{ 7.5, 3.75 };
		pix::Grad const srcGrad
			{ (float)srcSpot.row(), (float)srcSpot.col() };

		// Casting into 3D vectors (add a 3rd component identically zero)
		using namespace quadloco;
		engabra::g3::Vector const vecRowCol{ cast::vector(srcRowCol) };
		engabra::g3::Vector const vecSpot{ cast::vector(srcSpot) };
		engabra::g3::Vector const vecGrad{ cast::vector(srcGrad) };

		// Casting back into 2D (ignores the third component)
		dat::RowCol const dstRowCol{ cast::datRowCol(vecRowCol) };
		dat::Spot const dstSpot{ cast::datSpot(vecSpot) };
		pix::Grad const dstGrad{ cast::pixGrad(vecSpot) };

		// [DoxyExample01]

		if (! engabra::g3::nearlyEquals(vecRowCol[2], 0.))
		{
			oss << "Failure of vec[2]=0 RowCol test\n";
		}
		if (! engabra::g3::nearlyEquals(vecSpot[2], 0.))
		{
			oss << "Failure of vec[2]=0 Spot test\n";
		}
		if (! engabra::g3::nearlyEquals(vecGrad[2], 0.))
		{
			oss << "Failure of vec[2]=0 Grad test\n";
		}

		if (! nearlyEquals(dstRowCol, srcRowCol))
		{
			oss << "Failure of dst.from.vec RowCol test\n";
			oss << "src: " << srcRowCol << '\n';
			oss << "dst: " << dstRowCol << '\n';
		}
		if (! nearlyEquals(dstSpot, srcSpot))
		{
			oss << "Failure of dst.from.vec Spot test\n";
			oss << "src: " << srcSpot << '\n';
			oss << "dst: " << dstSpot << '\n';
		}
		if (! nearlyEquals(dstGrad, srcGrad))
		{
			oss << "Failure of dst.from.vec Grad test\n";
			oss << "src: " << srcGrad << '\n';
			oss << "dst: " << dstGrad << '\n';
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

