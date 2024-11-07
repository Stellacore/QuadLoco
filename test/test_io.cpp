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
\brief Unit tests (and example) code for quadloco::io
*/


#include "datGrid.hpp"
#include "io.hpp"
#include "pix.hpp"

#include <algorithm>
#include <cstdio>
#include <iostream>
#include <sstream>


namespace
{
	//! Check pgm image i/o
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		// example starting with a floating point grid
		using namespace quadloco;
		dat::SizeHW const hwSize{ 2u, 3u };
		dat::Grid<pix::fpix_t> fGrid{ hwSize };
		fGrid(0, 0) = -1.75;
		fGrid(0, 1) = -1.50;
		fGrid(0, 2) = -1.25;
		fGrid(1, 0) =  1.25;
		fGrid(1, 1) =  1.50;
		fGrid(1, 2) =  1.75;
		dat::Span const fSpan{ -2., 2. };

		// convert to classic uint8_t image
		dat::Grid<uint8_t> const uGridExp{ pix::uGrid8(fGrid, fSpan) };

		// write to PGM file
//		std::filesystem::path tmpFnamePgm("/tmp/quadloco_test_io.pgm");
std::filesystem::path tmpFnamePgm("./quadloco_test_io.pgm");
		bool const okayWrite{ io::writePGM(tmpFnamePgm, uGridExp) };

		// read from PGM file
		dat::Grid<uint8_t> const uGridGot{ io::readPGM(tmpFnamePgm) };
		bool const okayRead{ uGridGot.isValid() };

		// [DoxyExample01]

		if (! okayWrite)
		{
			oss << "Failure of write test\n";
		}

		if (! okayRead)
		{
			oss << "Failure of read test\n";
		}

		bool const samePixels
			{ std::equal
				(uGridExp.cbegin(), uGridExp.cend(), uGridGot.cbegin())
			};
		if (! samePixels)
		{
			oss << "Failure of reload pixel test\n";
			oss << "tmpFnamePgm: " << tmpFnamePgm << '\n';
			oss  << uGridExp.infoStringContents("uGridExp", "%4u") << '\n';
			oss  << uGridGot.infoStringContents("uGridGot", "%4u") << '\n';
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

