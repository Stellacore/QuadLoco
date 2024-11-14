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
\brief Unit tests (and example) code for quadloco::dat::Vec2D
*/


#include "datVec2D.hpp"

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
		using namespace quadloco;
		constexpr dat::Vec2D const aNull{};
		if ( isValid(aNull))
		{
			oss << "Failure of aNull test(0)\n";
			oss << "aNull: " << aNull << '\n';
		}

		// [DoxyExample00]
	
		using namespace quadloco::dat;
		Vec2D const vecA{ 3., 4. };
		Vec2D const vecB{ -4., 3. };

		Vec2D const gotUnitA{ direction(vecA) };
		double const gotMagA{ magnitude(vecA) };
		double const gotDotAB{ dot(vecA, vecB) };
		double const gotOuterAB{ outer(vecA, vecB) };

		Vec2D const gotAddAB{ vecA + vecB };
		Vec2D const gotSubAB{ vecA - vecB };

		// [DoxyExample00]

		double const expMagA{ std::hypot(3., 4.)  };
		Vec2D const expUnitA{ (1./expMagA)*3., (1./expMagA)*4. };
		double const expDotAB{ 3.*(-4.) + 4.*3. };
		double const expOuterAB{ 3.*3. - (-4.)*4. };
		Vec2D const expAddAB{ 3. + (-4.), 4.+3. };
		Vec2D const expSubAB{ 3. - (-4.), 4.-3. };

		if (! engabra::g3::nearlyEquals(gotMagA, expMagA))
		{
			oss << "Failure of gotMagA test(0)\n";
		}
		if (! nearlyEquals(gotUnitA, expUnitA))
		{
			oss << "Failure of gotUnitA test(0)\n";
		}
		if (! engabra::g3::nearlyEquals(gotDotAB, expDotAB))
		{
			oss << "Failure of gotDotAB test(0)\n";
		}
		if (! engabra::g3::nearlyEquals(gotOuterAB, expOuterAB))
		{
			oss << "Failure of gotOuterAB test(0)\n";
		}
		if (! nearlyEquals(gotAddAB, expAddAB))
		{
			oss << "Failure of gotAddAB test(0)\n";
		}
		if (! nearlyEquals(gotSubAB, expSubAB))
		{
			oss << "Failure of gotSubAB test(0)\n";
		}

	}

	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		using namespace quadloco;
		constexpr dat::Vec2D const aNull{};

		// [DoxyExample01]


		// [DoxyExample01]
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

