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
\brief Unit tests (and example) code for quadloco::obj::QuadTarget
*/


#include "objQuadTarget.hpp"

#include <iostream>
#include <sstream>
#include <vector>


namespace
{
	//! Examples for documentation
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		// construct a null instance
		quadloco::obj::QuadTarget const nullQuadTgt{};
		bool const nullIsOkay{ (false == isValid(nullQuadTgt)) };

		// construct with subpixel row,col order
		constexpr double fullEdgeMag{ .125 };
		constexpr double halfEdgeMag{ .5 * fullEdgeMag };
		quadloco::obj::QuadTarget const origQuadTgt{ fullEdgeMag };

		// radius to all outer corners is the same
		constexpr double expRadOuter{ std::sqrt(2.) * halfEdgeMag };
		double const gotRadOuter{ origQuadTgt.radiusOuter() };
		// radius to all midside (background) corners is same
		constexpr double expRadInner{ halfEdgeMag };
		double const gotRadInner{ origQuadTgt.radiusInner() };
		// center at origin
		quadloco::dat::Spot const expCenter{ 0., 0. };
		quadloco::dat::Spot const gotCenter{ origQuadTgt.centerLoc() };
		std::array<quadloco::dat::Spot, 4u>
			const gotCornerLocs{ origQuadTgt.cornerLocs() };

		// copy construction
		std::vector<quadloco::obj::QuadTarget> const copyQuadTgts
			{ origQuadTgt, origQuadTgt, origQuadTgt };

		// output operations
		std::ostringstream msg;
		quadloco::obj::QuadTarget const copyQuadTgt{ copyQuadTgts.back() };
		msg << copyQuadTgt << '\n';

		// get radiometric values from inside the target
		constexpr double partEdgeMag{ .8 * halfEdgeMag };
		quadloco::dat::Spot const spotRT{  partEdgeMag,  partEdgeMag };
		quadloco::dat::Spot const spotLT{ -partEdgeMag,  partEdgeMag };
		quadloco::dat::Spot const spotLB{ -partEdgeMag, -partEdgeMag };
		quadloco::dat::Spot const spotRB{  partEdgeMag, -partEdgeMag };
		float const expValInRT{ 0.f };
		float const expValInLT{ 1.f };
		float const expValInLB{ 0.f };
		float const expValInRB{ 1.f };
		float const gotValInRT{ origQuadTgt.intensityAt(spotRT) };
		float const gotValInLT{ origQuadTgt.intensityAt(spotLT) };
		float const gotValInLB{ origQuadTgt.intensityAt(spotLB) };
		float const gotValInRB{ origQuadTgt.intensityAt(spotRB) };

		// [DoxyExample01]

		if (! engabra::g3::nearlyEquals(gotRadOuter, expRadOuter))
		{
			oss << "Failure of radius test\n";
			oss << "exp: " << expRadOuter << '\n';
			oss << "got: " << gotRadOuter << '\n';
		}

		if (! engabra::g3::nearlyEquals(gotRadInner, expRadInner))
		{
			oss << "Failure of radius test\n";
			oss << "exp: " << expRadInner << '\n';
			oss << "got: " << gotRadInner << '\n';
		}

		if (! nearlyEquals(gotCenter, expCenter))
		{
			oss << "Failure of radius test\n";
			oss << "exp: " << expCenter << '\n';
			oss << "got: " << gotCenter << '\n';
		}

		using quadloco::dat::Spot;
		std::array<Spot, 4u> gotDiffs;
		for (std::size_t nn{0u} ; nn < 4u ; ++nn)
		{
			Spot const & gotCornerLoc = gotCornerLocs[nn];
			gotDiffs[nn] = gotCornerLoc - gotCenter;
		}
		// Outer product structure
		struct OP
		{
			double theBiv{};

			inline
			explicit
			OP
				( Spot const & spotA
				, Spot const & spotB
				)
				: theBiv{ spotA[0]*spotB[1] - spotA[1]*spotB[0] }
			{ }

		}; // OP
		double const & expR = expRadOuter;
		double const expOP
			{ OP(expR*Spot{ 1., 0. }, expR*Spot{ 0., 1. }).theBiv };
		double const gotOPa{ OP(gotDiffs[0], gotDiffs[1]).theBiv };
		double const gotOPb{ OP(gotDiffs[1], gotDiffs[2]).theBiv };
		double const gotOPc{ OP(gotDiffs[2], gotDiffs[3]).theBiv };
		double const gotOPd{ OP(gotDiffs[3], gotDiffs[0]).theBiv };
		constexpr double opTol{ 4. * std::numeric_limits<double>::epsilon() };
		bool const sameDiffBivs
			{  engabra::g3::nearlyEquals(gotOPa, expOP, opTol)
			&& engabra::g3::nearlyEquals(gotOPb, expOP, opTol)
			&& engabra::g3::nearlyEquals(gotOPc, expOP, opTol)
			&& engabra::g3::nearlyEquals(gotOPd, expOP, opTol)
			};
		if (! sameDiffBivs)
		{
			using engabra::g3::io::fixed;
			oss << "Failure of corner difference bivector test\n";
			oss << " expOP: " << fixed(expOP) << '\n';
			oss << "gotOPa: " << fixed(gotOPa) << '\n';
			oss << "gotOPb: " << fixed(gotOPb) << '\n';
			oss << "gotOPc: " << fixed(gotOPc) << '\n';
			oss << "gotOPd: " << fixed(gotOPd) << '\n';
		}

		if (! nullIsOkay)
		{
			oss << "Failure of nullIsOkay test\n";
			oss << "nullQuadTgt: " << nullQuadTgt << '\n';
		}

		if (msg.str().empty())
		{
			oss << "Failure of op<<() test\n";
		}

		if (! (gotValInRT == expValInRT))
		{
			oss << "Failure of value in RT test\n";
			oss << "exp: " << expValInRT << '\n';
			oss << "got: " << gotValInRT << '\n';
		}
		if (! (gotValInLT == expValInLT))
		{
			oss << "Failure of value in LT test\n";
			oss << "exp: " << expValInLT << '\n';
			oss << "got: " << gotValInLT << '\n';
		}
		if (! (gotValInLB == expValInLB))
		{
			oss << "Failure of value in LB test\n";
			oss << "exp: " << expValInLB << '\n';
			oss << "got: " << gotValInLB << '\n';
		}
		if (! (gotValInRB == expValInRB))
		{
			oss << "Failure of value in RB test\n";
			oss << "exp: " << expValInRB << '\n';
			oss << "got: " << gotValInRB << '\n';
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

