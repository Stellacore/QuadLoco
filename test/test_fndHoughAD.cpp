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


#include "datGrid.hpp"
#include "datRowCol.hpp"
#include "datSpot.hpp"
#include "pixGradel.hpp"
#include "pixgrid.hpp"
#include "pix.hpp"

#include <Engabra>

#include <iostream>
#include <sstream>


namespace quadloco
{

namespace dat
{

namespace cast
{
	//! Engabra Vector: [0,1] from spot, [2] set to zero.
	inline
	engabra::g3::Vector
	vector
		( dat::Spot const & spot
		)
	{
		return engabra::g3::Vector{ spot[0], spot[1], 0. };
	}

	//! Engabra Vector: [0,1] from gradient element, [2] set to zero.
	inline
	engabra::g3::Vector
	vector
		( pix::Gradel const & gradel
		)
	{
		return engabra::g3::Vector{ gradel[0], gradel[1], 0. };
	}

} // [cast]

} // [dat]

namespace fnd
{

	//! Description of line in raster space
	struct EdgeLine
	{
		//! Any point on the line
		dat::Spot const theAnyPntRC{};

		//! Direction of the (positive) gradient across the edge
		pix::Gradel const theGradelRC{};


		//! True if both the point location and gradent direction are valid
		inline
		bool
		isValid
			() const
		{
			return
				(  theAnyPntRC.isValid()
				&& theGradelRC.isValid()
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

			using dat::cast::vector;
			Vector const delta{ vector(rcLoc) - vector(theAnyPntRC) };
			double const rejection{ (delta* vector(theGradelRC)).theSca[0] };
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
				&& theGradelRC.nearlyEquals(other.theGradelRC)
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
				<< "theGradelRC: " << theGradelRC
				;

			return oss.str();
		}

	}; // EdgeLine

	inline
	dat::Grid<float>
	gridWithEdge
		( dat::SizeHW const & hwSize
		, EdgeLine const & edgeLine
		)
	{
		dat::Grid<float> pixGrid(hwSize);
		for (std::size_t row{0u} ; row < hwSize.high() ; ++row)
		{
			for (std::size_t col{0u} ; col < hwSize.wide() ; ++col)
			{
				dat::RowCol const rcLoc{ row, col };
				float pixValue{ 0. };
				if (edgeLine.rcInFront(rcLoc))
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


namespace quadloco
{

namespace fnd
{
	//! Circle containing bounding rectangle
	struct Circle
	{
		//! Circle center location (in [pix])
		dat::Spot const theCenter{};

		//! Circle radius (in [pix])
		double const theRadius{ engabra::g3::null<double>() };

		//! Circle circumscribing format
		inline
		static
		Circle
		circumScribing
			( dat::SizeHW const & hwSize
			)
		{
			dat::Spot const center{ hwSize.centerSpot() };
			double const radius{ .5*hwSize.diagonal() };
			return Circle{ center, radius };
		}

		//! True if this instance is valid
		inline
		bool
		isValid
			() const
		{
			return
				(  theCenter.isValid()
				&& engabra::g3::isValid(theRadius)
				);
		}

		//! True if this instance is nearly the same as other within tol
		inline
		bool
		nearlyEquals
			( Circle const & other
			, double const & tol = std::numeric_limits<double>::epsilon()
			) const
		{
			return
				(  theCenter.nearlyEquals(other.theCenter, tol)
				&& engabra::g3::nearlyEquals(theRadius, other.theRadius, tol)
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
				<< "theCenter: " << theCenter
				<< ' '
				<< "theRadius: " << theRadius
				;
			return oss.str();
		}

	}; // Circle

	struct CircleIntersect
	{
		dat::Spot const theCenterSpot{};
		double const theRadius{ engabra::g3::null<double>() };

		//! Solutions where ray intersects circle
		inline
		std::pair<dat::Spot, dat::Spot>
		spotSolutionPairFor
			( engabra::g3::Vector const & rayStart
			, engabra::g3::Vector const & rayDir
			) const
		{
			std::pair<dat::Spot, dat::Spot> solnPair
				{ dat::Spot{} , dat::Spot{} };


			// use engabra vectors for coding convenience
			using namespace engabra::g3;
			Vector const cpnt{ dat::cast::vector(theCenterSpot) };
			double const & rho = theRadius;

			// be sure direction is unitary (supresses quadratic coefficient)
			Vector const & spnt = rayStart;
			Vector const ddir{ direction(rayDir) };

			// quadratic equation components
			Vector const wvec{ spnt - cpnt };
			double const bHalf{ (wvec * ddir).theSca[0] };
			double const cc{ magSq(wvec) - rho*rho };
			double const lamMid{ -bHalf };
			double const radicand{ bHalf*bHalf - cc };

			// check for real roots (else return default null instances)
			if (! (radicand < 0.))
			{
				double const delta{ std::sqrt(radicand) };
				double const lamNeg{ lamMid - delta };
				double const lamPos{ lamMid + delta };

				Vector const xNeg{ spnt + lamNeg*ddir };
				Vector const xPos{ spnt + lamPos*ddir };

				solnPair.first  = dat::Spot{ xNeg[0], xNeg[1] };
				solnPair.second = dat::Spot{ xPos[0], xPos[1] };

				/*
				Vector const rPos{ xPos - cpnt };
				Vector const rNeg{ xNeg - cpnt };
				double const magPos{ magnitude(rPos) };
				double const magNeg{ magnitude(rNeg) };
				std::cout << "rPos: " << rPos << '\n';
				std::cout << "rNeg: " << rNeg << '\n';
				std::cout << "magPos: " << magPos << '\n';
				std::cout << "magNeg: " << magNeg << '\n';
				*/
			}

			return solnPair;
		}


	}; // CircleIntersect


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
			, Circle const & circle
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
			, Circle const & circle
			, double const & alpha
			)
		{
			double const dx{ spotOnCircle[0] - circle.theCenter[0] };
			double const dy{ spotOnCircle[1] - circle.theCenter[1] };
			double const delta{ (std::atan2(dy, dx) - alpha) };
/*
std::cout << "alphaFor: dx: " << dx << '\n';
std::cout << "alphaFor: dy: " << dy << '\n';
std::cout << "alphaFor: alpha: " << alpha << '\n';
std::cout << "alphaFor: delta: " << delta << '\n';
*/
			return delta;
		}

		inline
		static
		ParmAD
		from
			( EdgeLine const & edgeLine
			, Circle const & circle
			)
		{
			CircleIntersect const ci{ circle.theCenter, circle.theRadius };

			using namespace engabra::g3;
			using dat::cast::vector;
			Vector const rayStart{ vector(edgeLine.theAnyPntRC) };
			Vector const rayDir{ vector(edgeLine.theGradelRC) };

			// compute intersection points on circle
			std::pair<dat::Spot, dat::Spot> const solnPair
				{ ci.spotSolutionPairFor(rayStart, rayDir) };

/*
std::cout << "solnPair.1: " << solnPair.first << '\n';
std::cout << "solnPair.2: " << solnPair.second << '\n';
*/

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
		, quadloco::fnd::Circle const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::fnd::Circle const & item
		)
	{
		return item.isValid();
	}

	//! True if both items have very nearly the same values
	inline
	bool
	nearlyEquals
		( quadloco::fnd::Circle const & itemA
		, quadloco::fnd::Circle const & itemB
		, double const & tol = std::numeric_limits<double>::epsilon()
		)
	{
		return itemA.nearlyEquals(itemB, tol);
	}

} // [anon/global]

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
		quadloco::fnd::EdgeLine const expEdgeLine
			{ quadloco::dat::Spot{ 3., 4. }
			, quadloco::pix::Gradel{ 2., 4. }
			};

		// create an image with a strong edge
		quadloco::dat::SizeHW const hwSize{ 7u, 10u };
		quadloco::dat::Grid<float> const pixGrid
			{ quadloco::fnd::gridWithEdge(hwSize, expEdgeLine) };

		// compute Gradel image
		quadloco::dat::Grid<quadloco::pix::Gradel> const gradels
			{ quadloco::pix::grid::gradelGridFor(pixGrid) };

		// accumulate Gradel values into Hough A(lpha)-D(elta) buffer

		// extract maximum AD value
		quadloco::fnd::ParmAD const gotMaxAD
			{ // TODO
			};

		quadloco::fnd::Circle const circle
			{ quadloco::fnd::Circle::circumScribing(gradels.hwSize()) };
		quadloco::fnd::ParmAD const expMaxAD
			{ quadloco::fnd::ParmAD::from(expEdgeLine, circle) };

		// edge asociated with max AD peak
		quadloco::fnd::EdgeLine const gotEdgeLine{};

std::cout << pixGrid.infoStringContents("pixGrid", "%11.2f") << '\n';
std::cout << gradels.infoStringContents
	("gradels", quadloco::pix::Gradel::Formatter{}) << '\n';

		// [DoxyExample01]

		if (! nearlyEquals(gotEdgeLine, expEdgeLine))
		{
			oss << "Failure of edgeline test(1)\n";
			oss << "exp: " << expEdgeLine << '\n';
			oss << "got: " << gotEdgeLine << '\n';
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

