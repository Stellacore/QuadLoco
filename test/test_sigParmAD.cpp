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
\brief Unit tests (and example) code for quadloco::sig::ParmAD
*/


#include "sigParmAD.hpp"

#include <iostream>
#include <set>
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
		quadloco::sig::ParmAD const aNull{};
		bool const expValid{ false };
		bool const gotValid{ isValid(aNull) };

		// orthogonal line direction (e.g. lineDir is 1/4 RH turn from gradDir)
		quadloco::img::Grad const gradDir{ 1., 0. };
		quadloco::img::Vec2D<double> const expLineDir{ 0., 1. }; // RH 1/4 turn
		quadloco::img::Vec2D<double> const gotLineDir
			{ quadloco::sig::ParmAD::lineDirFromEdgeDir(gradDir) };

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
		quadloco::img::Spot const center{ 10., 20. };
		constexpr double radius{ 10. };
		quadloco::img::Circle const circle{ center, radius };

		// raster edge element
		using namespace quadloco;
		img::Spot const pixLoc{ 10.f, 21.f }; // shift in col has no effect
		img::Grad const pixGrad{ 1.f, 0.f }; // grad in row dir
		img::Edgel const edgel{ pixLoc, pixGrad };

		// the line segmeng of interest is perpendicular to edge gradient
		quadloco::img::Vec2D<double> const lineDir
			{ quadloco::sig::ParmAD::lineDirFromEdgeDir(edgel.gradient()) };

		// the Alpha,Delta parameters associate with the line segment
		static const double piHalf{ 2.*std::atan(1.) };
		static const double pi{ 2.*piHalf };
		quadloco::sig::ParmAD const expParmAD{ -piHalf, pi };
		quadloco::sig::ParmAD const gotParmAD
			{ quadloco::sig::ParmAD::from(edgel, circle) };


		// [DoxyExample01]

		if (! nearlyEquals(gotParmAD, expParmAD))
		{
			oss << "Failure of gotParmAD test(1)\n";
			oss << "exp: " << expParmAD << '\n';
			oss << "got: " << gotParmAD << '\n';
		}

	}

	//! Examples for documentation
	void
	test2
		( std::ostream & oss
		)
	{
		// useful test constants
		constexpr std::size_t numSamps{ 32u };
		constexpr float pi{ std::numbers::pi_v<float> };
		constexpr float piHalf{ .5 * pi };
		constexpr float piTwo{ 2. * pi };
		constexpr float dtheta{ piTwo / (float)numSamps };
		constexpr float drad{ 2.f / (float)numSamps };

		// [DoxyExample02]

		// bounding circle
		quadloco::img::Spot const center{ 0., 0. };
		constexpr double radius{ 1. };
		quadloco::img::Circle const circle{ center, radius };

		// raster edge element at center
		quadloco::pix::Spot const pixLoc{ 0.f, 0.f };

		// check that alpha values range [-pi <= alpha < pi)
		std::set<float> gotAlphas;
		// check that delta values range [0 <= alpha < 2.*pi)
		std::set<float> gotDeltas;
		for (float theta{0.f} ; theta < piTwo ; theta += dtheta)
		{
			using namespace quadloco;

			// rotate gradient direction through full circle
			img::Grad const pixGrad
				{ std::cosf(theta - piHalf)
				, std::sinf(theta - piHalf)
				};
			img::Edgel const edgel{ pixLoc, pixGrad };

			// the Alpha,Delta parameters associate with the line segment
			sig::ParmAD const gotParmAD
				{ sig::ParmAD::from(edgel, circle) };

			if (isValid(gotParmAD))
			{
				gotAlphas.insert(gotParmAD.alpha());
			}

			// adjust position in order to cover range of delta values
			for (float rad{-1.f} ; rad < 1.f ; rad += drad)
			{
				constexpr float rootHalf{ 1.f/std::numbers::sqrt2_v<float> };
				quadloco::pix::Spot const dSpot{ rad, rad };
				quadloco::pix::Spot const deltaLoc{ pixLoc + rootHalf*dSpot };
				img::Edgel const deltaEdgel{ deltaLoc, pixGrad };
				sig::ParmAD const deltaParmAD
					{ sig::ParmAD::from(deltaEdgel, circle) };
				if (isValid(deltaParmAD))
				{
					gotDeltas.insert(deltaParmAD.delta());
				}

				/*
				std::cout
					<< "theta,edgel,AD: " << engabra::g3::io::fixed(theta)
					<< ' ' << edgel.theGrad
					<< ' ' << deltaParmAD
					<< '\n';
				*/

			}

		}

		// [DoxyExample02]

		// check if as many alpha values as samples (since alpha unique here)
		if (! (numSamps == gotAlphas.size()))
		{
			oss << "Failure of gotAlphas size test\n";
			oss << "exp: " << numSamps << '\n';
			oss << "got: " << gotAlphas.size() << '\n';
		}
		if (! gotAlphas.empty())
		{
			float const maxAlpha{ *(gotAlphas.crbegin()) };
			if (! (maxAlpha < pi))
			{
				oss << "Failure of maxAlpha test(2)\n";
				oss << "     exp: should be strictly less than pi\n";
				oss << "     got: "
					<< engabra::g3::io::fixed(maxAlpha, 2u,8u) << '\n';
				oss << "pi-alpha: "
					<< engabra::g3::io::fixed(pi-maxAlpha, 2u,8u) << '\n';
			}

		}

		// check delta values (should all be very near pi)
		float const minDelta{ *(gotDeltas.cbegin()) };
		float const maxDelta{ *(gotDeltas.crbegin()) };
		if ( (minDelta < 0.f))
		{
			oss << "Failure of minDelta test\n";
			oss << "exp: " << 0.f << '\n';
			oss << "got: " << minDelta << '\n';
		}
		if (! (maxDelta < 2.f*pi))
		{
			oss << "Failure of maxDelta test\n";
			oss << "exp: " << (2.f*pi) << '\n';
			oss << "got: " << maxDelta << '\n';
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

