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
\brief Unit tests (and example) code for quadloco::fndHoughAB
*/


#include "cast.hpp"
#include "datCircle.hpp"
#include "datGrid.hpp"
#include "datRowCol.hpp"
#include "datSpot.hpp"
#include "fndHoughAD.hpp"
#include "pixEdgel.hpp"
#include "pixGrad.hpp"
#include "pixgrid.hpp"
#include "pix.hpp"

#include <Engabra>

#include <iostream>
#include <sstream>


namespace quadloco
{

namespace fnd
{

/*
	//! Description of line in raster space
	struct EdgeLine
	{
		//! Any point on the line
		dat::Spot const theAnyPntRC{};

		//! Direction of the (positive) gradient across the edge
		pix::Grad const theGradRC{};


		//! True if both the point location and gradent direction are valid
		inline
		bool
		isValid
			() const
		{
			return
				(  theAnyPntRC.isValid()
				&& theGradRC.isValid()
				);
		}

		//! True if location is in front of edge (relative to gradient)
		inline
		bool
		rcInFront
			( dat::Spot const & rcLoc
			) const
		{
			return (! rcInBack(rcLoc));
		}

		//! True if location is behind the edge (relative to gradient)
		inline
		bool
		rcInBack
			( dat::Spot const & rcLoc
			) const
		{
			// A bit wasteful to compute in 3D, but easy
			using namespace engabra::g3;

			using cast::vector;
			Vector const delta{ vector(rcLoc) - vector(theAnyPntRC) };
			double const rejection{ (delta* vector(theGradRC)).theSca[0] };
			return (rejection < 0.);
		}

		//! True if location is in front of edge (relative to gradient)
		inline
		bool
		rcInFront
			( dat::RowCol const & rowcol
			) const
		{
			return rcInFront	
				(dat::Spot{ (double)rowcol.row(), (double)rowcol.col() });
		}

		//! True if location is behind the edge (relative to gradient)
		inline
		bool
		rcInBack
			( dat::RowCol const & rowcol
			) const
		{
			return rcInBack
				(dat::Spot{ (double)rowcol.row(), (double)rowcol.col() });
		}

		//! True if components are same as those of other within tol
		inline
		bool
		nearlyEquals
			( EdgeLine const & other
			, double const & tol = std::numeric_limits<double>::epsilon()
			) const
		{
			return
				(  theAnyPntRC.nearlyEquals(other.theAnyPntRC)
				&& theGradRC.nearlyEquals(other.theGradRC)
				);
		}

		//! Descriptive information about this instance.
		inline
		std::string
		infoString
			( std::string const & title = {}
			) const
		{
			std::ostringstream oss;
			if (! title.empty())
			{
				oss << title << ' ';
			}
			oss
				<< "theAnyPntRC: " << theAnyPntRC
				<< ' '
				<< "theGradRC: " << theGradRC
				;

			return oss.str();
		}

	}; // EdgeLine
*/

	inline
	dat::Grid<float>
	gridWithEdge
		( dat::SizeHW const & hwSize
		, pix::Edgel const & edgel
		)
	{
		dat::Grid<float> pixGrid(hwSize);
		for (std::size_t row{0u} ; row < hwSize.high() ; ++row)
		{
			for (std::size_t col{0u} ; col < hwSize.wide() ; ++col)
			{
				dat::RowCol const rcLoc{ row, col };
				float pixValue{ 0. };
				if (edgel.rcInFront(rcLoc))
				{
					pixValue = 1.;
				}
				pixGrid(row, col) = pixValue;
			}
		}
		return std::move(pixGrid);
	}

} // [fnd]

} // [quadloco]


/*
namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::fnd::EdgeLine const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::fnd::EdgeLine const & item
		)
	{
		return item.isValid();
	}

	//! True if both items have very nearly the same values
	inline
	bool
	nearlyEquals
		( quadloco::fnd::EdgeLine const & itemA
		, quadloco::fnd::EdgeLine const & itemB
		, double const & tol = std::numeric_limits<double>::epsilon()
		)
	{
		return itemA.nearlyEquals(itemB, tol);
	}

} // [anon/global]
*/


namespace quadloco
{

namespace fnd
{
	//! Hough parameter space location
	struct ParmAD
	{
		double const theAlpha{ engabra::g3::null<double>() };
		double const theDelta{ engabra::g3::null<double>() };

		//! Angle (from circle center) to line seg start on cicle
		inline
		static
		double
		alphaFor
			( dat::Spot const & spotOnCircle
			, dat::Circle const & circle
			)
		{
			double const dx{ spotOnCircle[0] - circle.theCenter[0] };
			double const dy{ spotOnCircle[1] - circle.theCenter[1] };
			double const alpha{ std::atan2(dy, dx) };
/*
std::cout << "alphaFor: dx: " << dx << '\n';
std::cout << "alphaFor: dy: " << dy << '\n';
std::cout << "alphaFor: alpha: " << alpha << '\n';
*/
			return alpha;
		}

		//! Angle (from circle center) to line seg end on cicle
		inline
		static
		double
		deltaFor
			( dat::Spot const & spotOnCircle
			, dat::Circle const & circle
			, double const & alpha
			)
		{
			double const dx{ spotOnCircle[0] - circle.theCenter[0] };
			double const dy{ spotOnCircle[1] - circle.theCenter[1] };
			double const delta{ (std::atan2(dy, dx) - alpha) };
/*
std::cout << "deltaFor: dx: " << dx << '\n';
std::cout << "deltaFor: dy: " << dy << '\n';
std::cout << "deltaFor: alpha: " << alpha << '\n';
std::cout << "deltaFor: delta: " << delta << '\n';
*/
			return delta;
		}

		inline
		static
		ParmAD
		from
			( pix::Edgel const & edgel
			, dat::Circle const & circle
			)
		{
			using namespace engabra::g3;

			// compute intersection points on circle

			quadloco::dat::Vec2D<double> const lineDir
				{ quadloco::fnd::lineDirFromEdgeDir(edgel.theGrad) };

			dat::CircleIntersector const intersector{ circle };
			std::pair<dat::Spot, dat::Spot> const solnPair
{};//				{ intersector(edgel.theSpot, lineDir) };

			// compute alpha,delta values for intersection points
			double const alpha{ alphaFor(solnPair.first, circle) };
			double const delta{ deltaFor(solnPair.second, circle, alpha) };

			/*
			// check
			Vector const cpnt{ vector(circle.theCenter) };
			double const chkPos{ magnitude(vector(solnPair.first) - cpnt) };
			double const chkNeg{ magnitude(vector(solnPair.second) - cpnt) };

			std::cout << "chkPos: " << chkPos << '\n';
			std::cout << "chkNeg: " << chkNeg << '\n';
			*/

			return ParmAD{ alpha, delta };
		}

		//! True if this instance is valid
		inline
		bool
		isValid
			() const
		{
			return
				(  engabra::g3::isValid(theAlpha)
				&& engabra::g3::isValid(theDelta)
				);
		}

		//! True if this instance is nearly the same as other within tol
		inline
		bool
		nearlyEquals
			( ParmAD const & other
			, double const & tol = std::numeric_limits<double>::epsilon()
			) const
		{
			return
				(  engabra::g3::nearlyEquals(theAlpha, other.theAlpha, tol)
				&& engabra::g3::nearlyEquals(theDelta, other.theDelta, tol)
				);
		}

		//! Descriptive information about this instance.
		inline
		std::string
		infoString
			( std::string const & title = {}
			) const
		{
			std::ostringstream oss;
			if (! title.empty())
			{
				oss << title << ' ';
			}
			using engabra::g3::io::fixed;
			oss
				<< "theAlpha: " << fixed(theAlpha)
				<< ' '
				<< "theDelta: " << fixed(theDelta)
				;

			return oss.str();
		}

	}; // ParmAD

} // [fnd]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::fnd::ParmAD const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::fnd::ParmAD const & item
		)
	{
		return item.isValid();
	}

	//! True if both items have very nearly the same values
	inline
	bool
	nearlyEquals
		( quadloco::fnd::ParmAD const & itemA
		, quadloco::fnd::ParmAD const & itemB
		, double const & tol = std::numeric_limits<double>::epsilon()
		)
	{
		return itemA.nearlyEquals(itemB, tol);
	}

} // [anon/global]


namespace
{
	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		// generate grid (image) with a well defined edge
		quadloco::pix::Edgel const expEdgel
			{ quadloco::pix::Spot{ 3., 4. }
			, quadloco::pix::Grad{ 2., 4. }
			};

		// create an image with a strong edge
		quadloco::dat::SizeHW const hwSize{ 7u, 10u };
		quadloco::dat::Grid<float> const pixGrid
			{ quadloco::fnd::gridWithEdge(hwSize, expEdgel) };

		// expected configuration
		quadloco::dat::Circle const circle
			{ quadloco::dat::Circle::circumScribing(pixGrid.hwSize()) };
		quadloco::fnd::ParmAD const expMaxAD
			{ quadloco::fnd::ParmAD::from(expEdgel, circle) };

std::cout << '\n';
std::cout << "hwSize: " << hwSize << '\n';
std::cout << "circle: " << circle << '\n';
std::cout << "expEdgel: " << expEdgel << '\n';
std::cout << "expMaxAD: " << expMaxAD << '\n';
std::cout << '\n';


		// compute Grad image
		quadloco::dat::Grid<quadloco::pix::Grad> const grads
			{ quadloco::pix::grid::gradientGridFor(pixGrid) };

		// accumulate Grad values into Hough A(lpha)-D(elta) buffer

		// extract maximum AD value
		quadloco::fnd::ParmAD const gotMaxAD
			{ // TODO
			};

		// edge asociated with max AD peak
		quadloco::pix::Edgel const gotEdgel
			{ // TODO
			};

/*
std::cout << pixGrid.infoStringContents("pixGrid", "%11.2f") << '\n';
std::cout << grads.infoStringContents
	("grads", quadloco::pix::Grad::Formatter{}) << '\n';
*/

		// [DoxyExample01]

		if (! nearlyEquals(gotEdgel, expEdgel))
		{
			oss << "Failure of Edgel test(1)\n";
			oss << "exp: " << expEdgel << '\n';
			oss << "got: " << gotEdgel << '\n';
		}

		if (! isValid(circle))
		{
			oss << "Failure of valid circle test\n";
			oss << "circle: " << circle << '\n';
		}

		if (! nearlyEquals(gotMaxAD, expMaxAD))
		{
			oss << "Failure of parmMaxAD test(1)\n";
			oss << "exp: " << expMaxAD << '\n';
			oss << "got: " << gotMaxAD << '\n';
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

