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
\brief Unit tests (and example) code for quadloco::sig::EdgeInfo
*/


#include "sigEdgeInfo.hpp"

#include "imgVector.hpp"

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

		// test configuration
		img::Spot const deltaSpot{  -2., 4. };
		img::Spot const currSpot{  1.,  2. };
		img::Grad const currGrad{  .75, .25 };
		// other spot (for this test) having oppositely directed gradient
		img::Spot const otherSpot{  currSpot + deltaSpot };
		img::Grad const otherGrad{ -currGrad };

		// Tracking direction aligned with self, anti-aligned with other
		img::Vector<double> const expDir{ direction(currGrad - otherGrad) };

		// Track information in relation to this edgel
		img::Edgel const currEdgel{ currSpot, currGrad };
		sig::EdgeInfo currEdgeInfo{ currEdgel };

		// Update currEdgel tracking with other edgels
		img::Edgel const otherEdgel{ otherSpot, otherGrad };

		// update currEdgeInfo with other edgel candidates (here only 1 other)
		currEdgeInfo.considerOther(otherEdgel);
		double const gotWgt1{ currEdgeInfo.consideredWeight() };
		// typically many *different* others - but here only one available
		currEdgeInfo.considerOther(otherEdgel);
		double const gotWgt2{ currEdgeInfo.consideredWeight() };
		// gotWgt{1,2} reflect running weight after considerOther() calls

		// Pseudo-probability of belonging to a (quad target) radial edge
		double const gotWgt{ currEdgeInfo.consideredWeight() };
		// Estimated most likely gradient direction of this edge)
		img::Vector<double> const gotDir{ currEdgeInfo.consideredDirection() };
		// Estimated most-likely angle (aligned with gradient) for this edge
		// (convenience for ang::atan2(gotDir[1], gotDir[0]))
		double const gotAng{ currEdgeInfo.consideredAngle() };

		// [DoxyExample01]

		double const expAng{ ang::atan2(gotDir[1], gotDir[0]) };

		if (! (0. < gotWgt))
		{
			oss << "Failure of gotWgt test\n";
			oss << "exp: (0 < got) " << '\n';
			oss << "got: " << gotWgt << '\n';
		}
		if (! engabra::g3::nearlyEquals(gotWgt2, (2.*gotWgt1)))
		{
			oss << "Failure of gotWgt[1,2] test\n";
			oss << "got1: " << gotWgt1 << '\n';
			oss << "got2: " << gotWgt2 << '\n';
		}

		double const tol{ 4. * std::numeric_limits<double>::epsilon() };
		if (! nearlyEquals(gotDir, expDir, tol))
		{
			oss << "Failure of gotDir test\n";
			oss << "exp: " << expDir << '\n';
			oss << "got: " << gotDir << '\n';
		}

		if (! ang::nearlySameAngle(gotAng, expAng))
		{
			oss << "Failure of gotAng test\n";
			oss << "exp: " << expAng << '\n';
			oss << "got: " << gotAng << '\n';
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

