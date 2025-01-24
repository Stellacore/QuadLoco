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
\brief Contains example application demoFilter
*/


#include "io.hpp"
#include "opsgrid.hpp"
#include "rasGrid.hpp"

#include <filesystem>
#include <iostream>


namespace quadloco
{

} // [quadloco]


/*! \brief Demonstrate application of filter using ras:: ops:: capabilities.
*/
int
main
	( int argc
	, char * argv[]
	)
{
	if (! (2 < argc))
	{
		std::cerr << '\n';
		std::cerr << "Run sum-sq-diff filter on input PGM file\n";
		std::cerr << '\n';
		std::cerr << "Usage: <progname> <InputPGM> <OutputPGM>\n";
		std::cerr << '\n';
		return 1;
	}
	std::size_t ndx{ 1u };
	std::filesystem::path const srcPath(argv[ndx++]);
	std::filesystem::path const outPath(argv[ndx++]);

	using namespace quadloco;

	// load image
	ras::Grid<std::uint8_t> const srcGrid{ io::readPGM(srcPath) };
	ras::Grid<float> const useGrid{ ras::grid::realGridOf<float>(srcGrid, 0) };

	// smooth source signal
	ras::Grid<float> const softGrid
		{ ops::grid::smoothGridFor<float>(useGrid, 5u, 2.5) };

	// create filtered image (here SSD)
	ras::SizeHW const hwBox{ 5u, 5u };
	ras::Grid<float> const outGrid
		{ ops::grid::sumSquareDiffGridFor<float>(softGrid, hwBox) };

	// save (enlarged) image with center drawn
	bool const okaySave{ io::writeStretchPGM(outPath, outGrid) };

	std::cout << '\n';
	std::cout << "load : " << srcPath << ' ' << srcGrid << '\n';
	std::cout << "save : " << outPath << ' ' << outGrid << '\n';
	std::cout << "okaySave: " << okaySave << '\n';
	std::cout << '\n';

	return 0;
}







