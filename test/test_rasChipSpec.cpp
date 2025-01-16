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
\brief Unit tests (and example) code for quadloco::dat::Chip
*/


#include "rasChipSpec.hpp"
#include "rasgrid.hpp"
#include "rasGrid.hpp"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>


namespace tst
{
	using NdxRC = std::pair<std::size_t, std::size_t>;
	constexpr NdxRC nullNdxRC{ -1, -1 };

	//! Check non-null values for (ndxGrid == row,col) rtn non-null count
	inline
	std::size_t
	checkNdxGrid
		( std::ostream & oss
		, quadloco::ras::Grid<NdxRC> const & ndxGrid
		, std::string const & tname
		)
	{
		std::size_t count{ 0u };
		std::ostringstream msg;
		bool okay{ true };
		for (std::size_t row{0u} ; okay && (row < ndxGrid.high()) ; ++row)
		{
			for (std::size_t col{0u} ; okay && (col < ndxGrid.wide()) ; ++col)
			{
				NdxRC const expNdxRC{ row, col };
				NdxRC const & gotNdxRC = ndxGrid(row, col);
				bool const isValid(nullNdxRC != gotNdxRC);
				if (isValid)
				{
					++count;
					okay = (gotNdxRC == expNdxRC);
					if (! okay)
					{
						if (msg.str().empty())
						{
							msg << "Failure of " << tname
								<< " NdxRC equivalent test\n";
						}
						NdxRC const difNdxRC
							{ gotNdxRC.first - expNdxRC.first
							, gotNdxRC.second - expNdxRC.second
							};
						msg << "exp: " << std::setw(4u) << expNdxRC.first
								<< ' ' << std::setw(4u) << expNdxRC.second
							<< "  "
							<< "got: " << std::setw(4u) << gotNdxRC.first
								<< ' ' << std::setw(4u) << gotNdxRC.second
							<< "  "
							<< "dif: " << std::setw(4u) << difNdxRC.first
								<< ' ' << std::setw(4u) << difNdxRC.second
							<< '\n';
					}
				}
			}
		}
		oss << msg.str();
		return count;
	}

} // [anon]

namespace
{
	//! Check math
	void
	test0
		( std::ostream & oss
		)
	{
		using namespace quadloco;

		ras::SizeHW const fullSizeHW{  5u, 10u };
		ras::SizeHW const chipSizeHW{  2u,  3u };
		ras::RowCol const origIsIn{ 3u, 7u }; // most right/bottom that fits
		ras::RowCol const origOut1{ 4u, 7u }; // most right/bottom that fits
		ras::RowCol const origOut2{ 3u, 8u }; // most right/bottom that fits

		ras::ChipSpec const specIsIn{ origIsIn, chipSizeHW };
		ras::ChipSpec const specOut1{ origOut1, chipSizeHW };
		ras::ChipSpec const specOut2{ origOut2, chipSizeHW };

		bool const okayIsIn{ specIsIn.fitsInto(fullSizeHW) };
		if (! okayIsIn)
		{
			oss << "Failure of IsIn test(0)\n";
		}

		bool const okayOut1{ ! specOut1.fitsInto(fullSizeHW) };
		if (! okayOut1)
		{
			oss << "Failure of Out1 test(0)\n";
		}

		bool const okayOut2{ ! specOut2.fitsInto(fullSizeHW) };
		if (! okayOut2)
		{
			oss << "Failure of Out2 test(0)\n";
		}
	}


	//! Basic pixel level manipulation
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		// arbitrary "full-size" area
		quadloco::ras::SizeHW const fullHW{ 2160u, 4096u };

		// set a couple location values (for testing below)
		quadloco::ras::Grid<std::string> fullData(fullHW);
		std::fill(fullData.begin(), fullData.end(), "");
		std::string const expTL("Full:1000,2000"); // TL of raster chip
		std::string const expBL("Full:1039,2000"); // BL of raster chip
		std::string const expBR("Full:1039,2059"); // BR of raster chip
		std::string const expTR("Full:1000,2059"); // TR of raster chip
		fullData(1000u, 2000u) = expTL;
		fullData(1039u, 2000u) = expBL;
		fullData(1039u, 2059u) = expBR;
		fullData(1000u, 2059u) = expTR;

		// define a chip with which to access a sub area
		quadloco::ras::RowCol const chipRC{ 1000u, 2000u };
		quadloco::ras::SizeHW const chipHW{ 40u, 60u };
		quadloco::ras::ChipSpec const chipSpec{ chipRC, chipHW };

		// full pixel access via chipSpec index lookup
		// use chip for read cell access from the underlying full image
		std::string const gotTL = fullData(chipSpec.fullRowColFor( 0u,  0u));
		std::string const gotBL = fullData(chipSpec.fullRowColFor(39u,  0u));
		std::string const gotBR = fullData(chipSpec.fullRowColFor(39u, 59u));
		std::string const gotTR = fullData(chipSpec.fullRowColFor( 0u, 59u));

		// [DoxyExample01]

		// check full pixel access via chipSpec index lookup
		bool const okayFromFull
			{  (gotTL == expTL)
			&& (gotBL == expBL)
			&& (gotBR == expBR)
			&& (gotTR == expTR)
			};
		if (! okayFromFull)
		{
			oss << "Failure of chip read test(1)\n";
			oss << "expTL: '" << expTL << "'  "
				<< "gotTL: '" << gotTL << "'\n";
			oss << "expBL: '" << expBL << "'  "
				<< "gotBL: '" << gotBL << "'\n";
			oss << "expBR: '" << expBR << "'  "
				<< "gotBR: '" << gotBR << "'\n";
			oss << "expTR: '" << expTR << "'  "
				<< "gotTR: '" << gotTR << "'\n";
		}
	}


	//! Extraction of chip from full image
	void
	test2
		( std::ostream & oss
		)
	{
		// [DoxyExample02]

		// arbitrary "full-size" area
		quadloco::ras::SizeHW const fullHW{ 10u, 15u };

		// construct a full size raster
		using NdxRC = std::pair<std::size_t, std::size_t>;
		quadloco::ras::Grid<NdxRC> fullData(fullHW);
		// fill full data with (rowFullNdx,colFullNdx) pairs
		for (std::size_t rFull{0u} ; rFull < fullHW.high() ; ++rFull)
		{
			for (std::size_t cFull{0u} ; cFull < fullHW.wide() ; ++cFull)
			{
				fullData(rFull, cFull) = NdxRC{ rFull, cFull };
			}
		}

		// specify a chip raster relative to the full size grid
		quadloco::ras::RowCol const chipRC{ 3u, 8u }; // starting here
		quadloco::ras::SizeHW const chipHW{ 4u, 6u }; // of this size
		quadloco::ras::ChipSpec const chipSpec{ chipRC, chipHW };

		// true if chip fits inside full size
		bool const isContained{ chipSpec.fitsInto(fullHW) };

		// fill chip raster with data from the full size grid
		quadloco::ras::Grid<NdxRC> const chipData
			{ quadloco::ras::grid::subGridValuesFrom(fullData, chipSpec) };
		bool const okayFill{ chipSpec.isValid() };

		// [DoxyExample02]

		if (! isContained)
		{
			oss << "Failure of isContained test(2)\n";
		}

		// check if chip data were extracted
		if (! okayFill)
		{
			oss << "Failure of okayFill into chip test(2)\n";
		}

		// at this point, chipData grid is full of full index NdxRC values
		// therefore, subtract the starting point (expRow0,expCol0) in
		// order to compare NdxValues directly with access row,col position
		quadloco::ras::SizeHW const testHW{ chipData.hwSize() };
		quadloco::ras::Grid<NdxRC> testData(testHW);
		for (std::size_t testRow{0u} ; testRow < testHW.high() ; ++testRow)
		{
			for (std::size_t testCol{0u} ; testCol < testHW.wide() ; ++testCol)
			{
				NdxRC const chipNdx{ chipData(testRow, testCol) };
				NdxRC const testNdx
					{ chipNdx.first - chipSpec.srcRowBeg()
					, chipNdx.second - chipSpec.srcColBeg()
					};
				testData(testRow, testCol) = testNdx;
			}
		}

		// check that indices are set properly
		std::size_t const validCount
			{ tst::checkNdxGrid(oss, testData, "fillChip") };
		if (! (chipData.size() == validCount))
		{
			oss
				<< "\n\n=== full(2):\n"
				<< fullData.infoStringContents("", " (%2d,%2d)")
				<< "\n======\n"
				<< "--- chip(2):\n"
				<< testData.infoStringContents("test(2):\n", " (%2d,%2d)")
				<< "\n------\n"
				<< "testSpec: " << chipSpec << '\n'
				<< "validCount: " << validCount << '\n'
				;
		}
	}


	//! Filling section of full image from chip data
	void
	test3
		( std::ostream & oss
		)
	{
		// [DoxyExample03]

		// arbitrary "full-size" area
		quadloco::ras::SizeHW const fullHW{ 10u, 15u };

		// construct a full size raster
		using NdxRC = std::pair<std::size_t, std::size_t>;
		constexpr NdxRC nullNdxRC{ -1, -1 };
		quadloco::ras::Grid<NdxRC> fullData(fullHW);
		std::fill(fullData.begin(), fullData.end(), nullNdxRC);

		// specify a chip raster relative to the full size grid
		quadloco::ras::RowCol const chipRC{ 3u, 8u }; // starting here
		quadloco::ras::SizeHW const chipHW{ 4u, 6u }; // of this size
		quadloco::ras::ChipSpec const chipSpec{ chipRC, chipHW };

		// true if chip fits inside full size
		bool const isContained{ chipSpec.fitsInto(fullHW) };

		// create a chip ...
		quadloco::ras::Grid<NdxRC> chipData(chipHW);
		// ... and fill with data that should match fullData row/col
		for (std::size_t chipRow{0u} ; chipRow < chipHW.high() ; ++chipRow)
		{
			for (std::size_t chipCol{0u} ; chipCol < chipHW.wide() ; ++chipCol)
			{
				// expected position of (chipRow,chipCol) inside fullData
				NdxRC const fullNdxRC
					{ chipRow + chipSpec.srcRowBeg()
					, chipCol + chipSpec.srcColBeg()
					};
				chipData(chipRow, chipCol) = fullNdxRC;
			}
		}

		// use chip spec to set associated cells in full data structure
		bool const okayFill
			{ quadloco::ras::grid::setSubGridInside
				(&fullData, chipData, chipSpec.srcOrigRC())
			};

		// [DoxyExample03]

		if (! isContained)
		{
			oss << "Failure of isContained test(3)\n";
		}

		// check if chip data were extracted
		if (! okayFill)
		{
			oss << "Failure of okayFill into full test(3)\n";
		}

		// check that indices are set properly
		std::size_t const validCount
			{ tst::checkNdxGrid(oss, fullData, "fillFull") };
		if (! (chipData.size() == validCount))
		{
			oss
				<< "\n\n=== chip(3):\n"
				<< chipData.infoStringContents("", " (%2d,%2d)")
				<< "\n======\n"
				<< "--- full(3):\n"
				<< fullData.infoStringContents("", " (%2d,%2d)")
				<< "\n------\n"
				<< "chipSpec: " << chipSpec << '\n'
				<< "validCount: " << validCount << '\n'
				;
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
	test3(oss);

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

