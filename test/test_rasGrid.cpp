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
\brief Unit tests (and example) code for quadloco::ras::Grid
*/


#include "QuadLoco/rasGrid.hpp"

#include <algorithm>
#include <functional>
#include <iostream>
#include <numeric>
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

		// construct a grid of arbitrary data type
		struct SomeType
		{
			// use this function in custom lambda below
			inline
			std::string
			infoString
				() const
			{
				return std::string("WhatEver");
			}
		};
		quadloco::ras::Grid<SomeType> const someGrid(5u, 6u);

		// output contents with custom formatting function
		// note that default separator between fields is a space
		std::ostringstream msg;
		msg << someGrid.infoStringContents
			( "someGrid"  // title string
			, [] (SomeType const & element) { return element.infoString(); }
			);

		// [DoxyExample00]

		std::string::size_type const end = std::string::npos;
		std::string::size_type pos = 0;
		std::size_t count{ 0u };
		std::string const str{ msg.str() };
		std::string const word{ SomeType{}.infoString() };
		while (end != (pos = str.find("WhatEver", pos)))
		{
			++count;
			pos += word.length();
		}
		if (! (someGrid.size() == count))
		{
			oss << "Failure of output cell count test\n";
			oss << "exp: " << someGrid.size() << '\n';
			oss << "got: " << count << '\n';
		}

	}

	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		// general raster grid storage and access - tiny image example
		quadloco::ras::SizeHW const hwSize{ 2u, 3u };
		quadloco::ras::Grid<float> grid(hwSize);

		// use iterators to fill values
		constexpr float fillVal{ 17.f/8.f };
		std::fill(grid.begin(), grid.end(), fillVal);

		// set a couple cell values explicitly
		constexpr float dVal{ 2.5f };
		using quadloco::ras::RowCol;
		grid(RowCol{ 0u, 0u }) = dVal;
		grid(1u, 1u) = dVal;
		grid(0u, 2u) = grid(0u, 0u);

		// make a deep copy (in place of op==() or copy ctor())
		using FGrid = quadloco::ras::Grid<float>;
		FGrid const copy{ FGrid::copyOf(grid) };

		// constant iterator access
		float const gotSum
			{ std::accumulate(copy.cbegin(), copy.cend(), 0.f) };
		float const expSum{ 3.f*dVal + 3.*fillVal };

		// output (ref infoStringContents() overloads for custom data types)
		std::ostringstream ostrm;
		ostrm << grid << '\n'; // puts grid.infoString() to stream
		// put grid cell contents to stream
		ostrm << grid.infoStringContents("gridValues\n", "%9.3f") << '\n';

		// [DoxyExample01]

		if (! (hwSize.high() == grid.high()))
		{
			oss << "Failure of grid.high() test\n";
		}
		if (! (hwSize.wide() == grid.wide()))
		{
			oss << "Failure of grid.wide() test\n";
		}

		if (! (gotSum == expSum)) // exact given binary exact test values.
		{
			oss << "Failure of grid sum test\n";
			oss << "expSum: " << expSum << '\n';
			oss << "gotSum: " << gotSum << '\n';
			oss << "grid: " << grid << '\n';
			oss << copy.infoStringContents("copy\n", "%6.3f") << '\n';
		}

		// exercise i/o functions

		// save contents to stream
		std::ostringstream save;
		save << grid.infoStringContents("", "%9.3f") << '\n';
		// load from stream (here into a double value grid)
		std::istringstream iss{ save.str() };
		std::string txt;
		std::size_t high, wide;
		std::size_t nc, nb;
		iss >> txt;  // skip "High,Wide" label
		iss >> high >> wide;
		iss >> txt;  // skip "Cells,Bytes" label
		iss >> nc >> nb;  // skip cell and byte size values
		quadloco::ras::SizeHW const hwLoad{ high, wide };
		quadloco::ras::Grid<double> load(hwLoad); // create data container
		quadloco::ras::Grid<double>::iterator inIter{ load.begin() };
		for (std::size_t nCell{0u} ; nCell < load.size() ; ++nCell)
		{
			iss >> *inIter++;
		}

		bool const loadOkay
			{ std::equal(grid.cbegin(), grid.cend(), load.cbegin()) };
		if (! loadOkay)
		{
			oss << "Failure of save/load test\n";
			oss << grid.infoStringContents("grid\n", "%9.3f") << '\n';
			oss << load.infoStringContents("load\n", "%9.3f") << '\n';
		}

	}

	//! Examples for documentation
	void
	test2
		( std::ostream & oss
		)
	{
		// [DoxyExample02]

		using namespace quadloco;

		// null grid should be (! nearlyEqual to all)
		ras::Grid<double> const null{};

		//! Grid of one size
		ras::Grid<double> gridA(2u, 1u);
		ras::Grid<double>::iterator itA{ gridA.begin() };
		*itA++ =  .5;
		*itA++ = 1.5;

		//! Grid of different size but same content
		ras::Grid<double> gridB(1u, 2u);
		ras::Grid<double>::iterator itB{ gridB.begin() };
		*itB++ =  .5;
		*itB++ = 1.5;

		bool showDetail{ false };

		// null not nearly equal to anything
		if (! (! nearlyEquals(null, null)))
		{
			oss << "Failure of null,null test\n";
			showDetail = true;
		}
		if (! (! nearlyEquals(gridA, null)))
		{
			oss << "Failure of gridA,null test\n";
			showDetail = true;
		}

		// non-null grid equal to self
		if (! nearlyEquals(gridA, gridA))
		{
			oss << "Failure of gridA,gridA test\n";
			showDetail = true;
		}

		// grids with same content but different sizes are not nearly same
		if (! (! nearlyEquals(gridA, gridB)))
		{
			oss << "Failure of gridA,gridB test\n";
			showDetail = true;
		}

		if (showDetail)
		{
			oss << gridA.infoStringContents("gridA", "%5.3f") << '\n';
			oss << gridB.infoStringContents("gridB", "%5.3f") << '\n';
		}

		// [DoxyExample02]

	}
}

//! Check behavior of NS
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

