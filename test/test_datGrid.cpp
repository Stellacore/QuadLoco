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
\brief Unit tests (and example) code for quadloco::dat::Grid
*/


#include "datGrid.hpp"

#include "QuadLoco"

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
		quadloco::dat::Grid<SomeType> const someGrid(5u, 6u);

		// output contents with custom formatting function
		// note that infoStringContents adds a space between elements
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
		quadloco::dat::SizeHW const hwSize{ 2u, 3u };
		quadloco::dat::Grid<float> grid(hwSize);

		// use iterators to fill values
		constexpr float fillVal{ 17.f/8.f };
		std::fill(grid.begin(), grid.end(), fillVal);

		// set a couple cell values explicitly
		constexpr float dVal{ 2.5f };
		using quadloco::dat::RowCol;
		grid(RowCol{ 0u, 0u }) = dVal;
		grid(1u, 1u) = dVal;
		grid(0u, 2u) = grid(0u, 0u);

		// return move copy
		using FGrid = quadloco::dat::Grid<float>;
		std::function<FGrid(FGrid const &)> const copyFunc
			{ [](FGrid const & orig)
				{
				FGrid copy(orig.hwSize());
				std::copy(orig.cbegin(), orig.cend(), copy.begin());
				return std::move(copy); // move assign
				}
			};
		FGrid const copy{ copyFunc(grid) };

		// constant iterator access
		float const gotSum
			{ std::accumulate(copy.cbegin(), copy.cend(), 0.f) };
		float const expSum{ 3.f*dVal + 3.*fillVal };

		// output
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
		quadloco::dat::SizeHW const hwLoad{ high, wide };
		quadloco::dat::Grid<double> load(hwLoad); // create data container
		quadloco::dat::Grid<double>::iterator inIter{ load.begin() };
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

