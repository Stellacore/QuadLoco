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
\brief Unit tests (and example) code for quadloco::img::Ray
*/


#include "QuadLoco/imgRay.hpp"

#include "QuadLoco/imgGrad.hpp"
#include "QuadLoco/imgSpot.hpp"

#include <iostream>
#include <sstream>


namespace
{
	//! Check basics
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample00]

		// check null pattern use
		quadloco::img::Ray const aNull{};
		bool const expIsValid{ false };
		bool const gotIsValid{ isValid(aNull) };

		// create ray from img:: space derived objects
		quadloco::img::Ray const origRay
			{ quadloco::img::Spot{ -1.25, 2.75 }
			, quadloco::img::Grad{   .75, -.25 }
			};
		quadloco::img::Ray const copyRay{ origRay };

		// [DoxyExample00]

		if (! (gotIsValid == expIsValid))
		{
			oss << "Failure of aNull test\n";
			oss << "aNull: " << aNull << '\n';
		}

		if (! nearlyEquals(copyRay, origRay))
		{
			oss << "Failure of copyRay test\n";
			oss << "exp: " << origRay << '\n';
			oss << "got: " << copyRay << '\n';
		}

	}

	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		// arbitrary ray
		quadloco::img::Ray const ray
			{ quadloco::img::Spot{ 2., 3. }
			, quadloco::img::Vector<double>{ 1., 2. } // normalized by ctor
			};

		// point projected onto ray
		quadloco::img::Vector<double> const expPnt{ 1., 7. };
		double const gotDistAlong{ ray.distanceAlong(expPnt) };
		double const gotDistFrom{ ray.distanceFrom(expPnt) };
		// reconstitute point location
		quadloco::img::Vector<double> const gotPnt
			{ ray.start()
			+ gotDistAlong * ray.direction()
			+ gotDistFrom * ray.orthoDirection()
			};

		// is point projection onto ray in front or behind start point
		quadloco::img::Spot const fwdPnt{ 1., 4. };
		quadloco::img::Spot const bckPnt{ 0., 3. };
		bool const gotIsFwd{ ray.isAhead(fwdPnt) };
		bool const gotIsBck{ ray.isBehind(bckPnt) };

		// [DoxyExample01]

		if (! (gotIsFwd && gotIsBck))
		{
			double const fwdDistAlong{ ray.distanceAlong(fwdPnt) };
			double const bckDistAlong{ ray.distanceAlong(bckPnt) };
			oss << "Failure of ahead/behind test\n";
			oss << "gotIsFwd: " << gotIsFwd << '\n';
			oss << "gotIsBck: " << gotIsBck << '\n';
			oss << "fwdDistAlong: " << fwdDistAlong << '\n';
			oss << "bckDistAlong: " << bckDistAlong << '\n';
		}

		// check orthoDirection
		double const expDot{ dot(ray.direction(), ray.orthoDirection()) };
		double const gotDot{ dot(ray.direction(), ray.orthoDirection()) };
		if (! engabra::g3::nearlyEquals(gotDot, expDot))
		{
			oss << "Failure of gotDot test\n";
			oss << "exp: " << expDot << '\n';
			oss << "got: " << gotDot << '\n';
		}

		// check point reconstruction from projection/rejection
		if (! nearlyEquals(gotPnt, expPnt))
		{
			oss << "Failure of gotPnt decomposition test\n";
			oss << "exp: " << expPnt << '\n';
			oss << "got: " << gotPnt << '\n';
			oss << "ray: " << ray << '\n';
			oss << "gotDistAlong: " << gotDistAlong << '\n';
			oss << " gotDistFrom: " << gotDistFrom << '\n';
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

