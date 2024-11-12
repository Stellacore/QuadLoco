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
	// A bit wasteful in extra computation time, and storage, but easy
	using namespace engabra::g3;

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

