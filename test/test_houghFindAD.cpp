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
\brief Unit tests (and example) code for quadloco::houghHoughAB
*/


#include "cast.hpp"
#include "datCircle.hpp"
#include "datGrid.hpp"
#include "datRowCol.hpp"
#include "datSpot.hpp"
#include "houghParmAD.hpp"
#include "pixEdgel.hpp"
#include "pixGrad.hpp"
#include "pixgrid.hpp"
#include "pix.hpp"

#include <Engabra>

#include <iostream>
#include <sstream>


namespace quadloco
{

namespace sim
{

	//! Create a grid with a strong edge that aligns with provided edgel
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

} // [sim]

} // [quadloco]



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

		// simulate an image with a very strong edge
		quadloco::dat::SizeHW const hwSize{ 7u, 10u };
		quadloco::dat::Grid<float> const pixGrid
			{ quadloco::sim::gridWithEdge(hwSize, expEdgel) };

		// compute Grad image
		quadloco::dat::Grid<quadloco::pix::Grad> const grads
			{ quadloco::pix::grid::gradientGridFor(pixGrid) };

		// expected configuration
		quadloco::dat::Circle const circle
			{ quadloco::dat::Circle::circumScribing(grads.hwSize()) };
		quadloco::hough::ParmAD const expMaxAD
			{ quadloco::hough::ParmAD::from(expEdgel, circle) };

std::cout << '\n';
std::cout << "hwSize: " << hwSize << '\n';
std::cout << "circle: " << circle << '\n';
std::cout << "expEdgel: " << expEdgel << '\n';
std::cout << "expMaxAD: " << expMaxAD << '\n';
std::cout << '\n';


		// accumulate Grad values into Hough A(lpha)-D(elta) buffer
		for (quadloco::dat::Grid<quadloco::pix::Grad>::const_iterator
			iter{grads.cbegin()} ; grads.cend() != iter ; ++iter)
		{
//			quadloco::dat::RowCol const rowcol{ grads.datRowColFor(iter) };
			quadloco::pix::Grad const & grad = *iter;
			quadloco::pix::Edgel const edgel
				{ quadloco::cast::pixSpot(grads.datRowColFor(iter))
				, grad
				};

			quadloco::hough::ParmAD const parmAD
				{ quadloco::hough::ParmAD::from(edgel, circle) };

std::cout << "parmAD: " << parmAD << '\n';

		}

		// extract maximum AD value
		quadloco::hough::ParmAD const gotMaxAD
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

