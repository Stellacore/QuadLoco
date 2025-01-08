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
\brief Program to inspect PGM file
*/


#include "io.hpp"
#include "rasGrid.hpp"
#include "sigutil.hpp"

#include <filesystem>
#include <iostream>


namespace quadloco
{
} // [quadloco]


/*! \brief Read PGM file header and data size
*/
int
main
	( int argc
	, char * argv[]
	)
{
	if (! (1 < argc))
	{
		std::cerr << '\n';
		std::cerr << "Report information about a PGM file\n";
		std::cerr << '\n';
		std::cerr << "Usage: <progname> <InputPGM>\n";
		std::cerr << '\n';
		return 1;
	}
	std::size_t ndx{ 1u };
	std::filesystem::path const srcPath(argv[ndx++]);

	using namespace quadloco;

	// load image
	ras::Grid<std::uint8_t> const srcGrid{ io::readPGM(srcPath) };

	// report size info
	std::cout << "srcGrid: " << srcGrid << '\n';
//	std::cout << srcGrid.infoStringContents("srcGrid:", "%4d") << '\n';

	return 0;
}


