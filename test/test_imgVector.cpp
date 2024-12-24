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
\brief Unit tests (and example) code for quadloco::img::Vector
*/


#include "imgVector.hpp"

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
		constexpr img::Vector<double> const aNull{};
		if ( isValid(aNull))
		{
			oss << "Failure of aNull test(0)\n";
			oss << "aNull: " << aNull << '\n';
		}

		// [DoxyExample00]

		img::Vector<double> const vecA{ 3., 4. };
		img::Vector<double> const vecB{ -4., 3. };

		img::Vector<double> const gotUnitA{ direction(vecA) };
		double const gotMagA{ magnitude(vecA) };
		double const gotDotAB{ dot(vecA, vecB) };
		double const gotOuterAB{ outer(vecA, vecB) };

		img::Vector<double> const gotAddAB{ vecA + vecB };
		img::Vector<double> const gotSubAB{ vecA - vecB };

		// [DoxyExample00]

		double const expMagA{ std::hypot(3., 4.)  };
		img::Vector<double> const expUnitA{ (1./expMagA)*3., (1./expMagA)*4. };
		double const expDotAB{ 3.*(-4.) + 4.*3. };
		double const expOuterAB{ 3.*3. - (-4.)*4. };
		img::Vector<double> const expAddAB{ 3. + (-4.), 4.+3. };
		img::Vector<double> const expSubAB{ 3. - (-4.), 4.-3. };

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

	//! Check basic operations
	void
	test1
		( std::ostream & oss
		)
	{
		using namespace quadloco;
		constexpr img::Vector<float> const aNull{};

		// [DoxyExample01]

		// dot/outer product conventions
		using Vec = quadloco::img::Vector<double>;
		double const gotDot{ dot(Vec{1., 0.}, Vec{0., 1. }) };
		double const gotOut{ outer(Vec{1., 0.}, Vec{0., 1. }) };
		double const expDot{ 0. };
		double const expOut{ 1. }; // +1 for righthand convention on outer
		if (! engabra::g3::nearlyEquals(gotDot, expDot))
		{
			oss << "Failure of gotDot test\n";
			oss << "exp: " << expDot << '\n';
			oss << "got: " << gotDot << '\n';
		}
		if (! engabra::g3::nearlyEquals(gotOut, expOut))
		{
			oss << "Failure of gotOut test\n";
			oss << "exp: " << expOut << '\n';
			oss << "got: " << gotOut << '\n';
		}

		// [DoxyExample01]
	}

	//! Check vector operators
	void
	test2
		( std::ostream & oss
		)
	{
		using namespace quadloco;

		// Construct a vector prependicular to staring vector
		img::Vector<float> const aVec{ 3., -2. };
		img::Vector<float> const perp{ ccwPerp(aVec) };

		// For the perpendicular, dot product with original is zero
		float const gotDot{ dot(aVec, perp) };
		float const expDot{ 0.f };

		// For the perpendicular, outer product with original is magSq
		float const gotOut{ outer(aVec, perp) };
		float const expOut{ magnitude(aVec) * magnitude(perp) };

		if (! engabra::g3::nearlyEquals(gotDot, expDot))
		{
			oss << "Failure of gotDot test\n";
			oss << "exp: " << expDot << '\n';
			oss << "got: " << gotDot << '\n';
		}

		if (! engabra::g3::nearlyEquals(gotDot, expDot))
		{
			oss << "Failure of gotOut test\n";
			oss << "exp: " << expOut << '\n';
			oss << "got: " << gotOut << '\n';
		}

		// [DoxyExample02]
		// [DoxyExample02]
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

