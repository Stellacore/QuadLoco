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
\brief Unit tests (and example) code for quadloco::pix namespace functions
*/


#include "pix.hpp"
#include "rasGrid.hpp"

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
		// [DoxyExample01]

		// create a small simple floating point imag
		using quadloco::pix::fpix_t;
		quadloco::ras::Grid<fpix_t> fGrid(quadloco::ras::SizeHW{ 5u, 1u });
		fGrid(0, 0) = 100.;
		fGrid(1, 0) = 101.;
		fGrid(2, 0) = 300.;
		fGrid(3, 0) = 500.-.001;
		fGrid(4, 0) = 500.;

		// expected working span of input imagery (for test case)
		quadloco::sig::Span const testSpan{ 101., 500. };

		// full span (that ensures all pixels are valid)
		quadloco::sig::Span const fullSpan
			{ quadloco::pix::fullSpanFor(fGrid) };

		// convert to classic uint8_t image
		quadloco::ras::Grid<uint8_t> const uGrid
			{ quadloco::pix::uGrid8(fGrid, testSpan) };

		//
		// Expected mapping of values given testSpan
		//

		// expected output values (relative to proportions of testSpan)
		using namespace quadloco::pix; // for the u8... types
		uint8_t const exp00{ u8Undr }; // 100. // under exposed
		uint8_t const exp10{ u8Dark }; // 101. // dark (but okay) exposure
		uint8_t const exp20{   128u }; // 200. // good exposure
		uint8_t const exp30{ u8Lite }; // 500.-.001 // lite (but okay) exposure
		uint8_t const exp40{ u8Over }; // 500. // over expossed

		uint8_t const & got00 = uGrid(0, 0);
		uint8_t const & got10 = uGrid(1, 0);
		uint8_t const & got20 = uGrid(2, 0);
		uint8_t const & got30 = uGrid(3, 0);
		uint8_t const & got40 = uGrid(4, 0);

		// [DoxyExample01]

		// check if full span contains all pixels
		using FIter = quadloco::ras::Grid<fpix_t>::const_iterator;
		bool allIn{ true };
		for (FIter iter{fGrid.cbegin()} ; fGrid.cend() != iter ; ++iter)
		{
			fpix_t const & pixVal = *iter;
			bool const isIn{ fullSpan.contains(pixVal) };
			allIn &= isIn;
		}
		if (! allIn)
		{
			oss << "Failure of all in fullSpan test\n";
		}

		bool allOkay{ true };
		allOkay &= (got00 == exp00);
		allOkay &= (got10 == exp10);
		allOkay &= (got20 == exp20);
		allOkay &= (got30 == exp30);
		allOkay &= (got40 == exp40);

		if (! allOkay)
		{
			oss << "Failure of radiometric remapping test\n";
			oss << fGrid.infoStringContents("fGrid", "%6.3f") << '\n';
			oss << uGrid.infoStringContents("uGrid", "%4d") << '\n';
			oss
				<< "exp00:" << std::setw(4u) << +exp00 << "  "
				<< "got00:" << std::setw(4u) << +got00 << '\n';
			oss
				<< "exp10:" << std::setw(4u) << +exp10 << "  "
				<< "got10:" << std::setw(4u) << +got10 << '\n';
			oss
				<< "exp20:" << std::setw(4u) << +exp20 << "  "
				<< "got20:" << std::setw(4u) << +got20 << '\n';
			oss
				<< "exp30:" << std::setw(4u) << +exp30 << "  "
				<< "got30:" << std::setw(4u) << +got30 << '\n';
			oss
				<< "exp40:" << std::setw(4u) << +exp40 << "  "
				<< "got40:" << std::setw(4u) << +got40 << '\n';
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

