//
// MIT License
//
// Copyright (c) 2025 Stellacore Corporation
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
\brief Unit tests (and example) code for quadloco::app::QuadCycle
*/


#include "QuadLoco/QuadLoco"

#include <Engabra>

#include <numbers>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>


namespace
{


	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		using namespace quadloco;

		// [DoxyExample01]

		// load real image chip as test data...
		std::filesystem::path const srcPath("./p5q5.pgm");
		ras::Grid<uint8_t> const pgmGrid{ io::readPGM(srcPath) };
		// ... and cast to floating point grid
		ras::Grid<float> const srcGrid
			{ ras::grid::realGridOf<float>(pgmGrid) };

		// location to be checked for quad azimuth symmetry
		img::Spot const evalCenter{ 24.4, 25.1 }; // from manual measurement

		// create quad azimuth cycle assessor
		double const evalRadius{ 7.0 }; // max radius of evaluation space
		double const evalMinRad{ 2.5 }; // min radius (skip if less than this)
		quadloco::app::AzimCycle const azimCycle
			(srcGrid, evalCenter, evalRadius, evalMinRad);

		// test if azimuth cycles are consistent with a quad pattern
		bool const isQuadish{ azimCycle.hasQuadTransitions() };

		// [DoxyExample01]

		if (! isQuadish)
		{
			using engabra::g3::io::fixed;
			oss << "Failure of isQuadish test\n";
			oss << "srcPath: " << srcPath << '\n';
			oss << "srcGrid: " << srcGrid << '\n';
			oss << "evalCenter: " << evalCenter << '\n';
			oss << "evalRadius: " << fixed(evalRadius) << '\n';
			oss << "evalMinRad: " << fixed(evalMinRad) << '\n';


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

