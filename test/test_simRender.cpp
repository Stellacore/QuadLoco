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
\brief Unit tests (and example) code for quadloco::obj::QuadTarget
*/


#include "io.hpp"
#include "objQuadTarget.hpp"
#include "pix.hpp"
#include "rasGrid.hpp"
#include "simConfig.hpp"
#include "simRender.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>


namespace
{
	//! Check centering / subpixel precision
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample00]

		//
		// Exact sampling (no oversampling or noise)
		//

		// simulation test configuration
		constexpr double edgeMag{ 1. };
		constexpr std::size_t numPix{ 2u };
		constexpr std::size_t numOverSample{ 0u }; // 0-> no over sampling

		// define a quad target object
		quadloco::obj::QuadTarget const objQuad
			( edgeMag
			, quadloco::obj::QuadTarget::None
		//	| quadloco::obj::QuadTarget::WithTriangle
		//	| quadloco::obj::QuadTarget::WithSurround
			);

		// configure camera and orientation such that
		// camera format exactly matches the objQuad target
		quadloco::sim::Config const config
			{ quadloco::sim::Config::faceOn(objQuad, numPix) };


		// render result
		using opt = quadloco::sim::Sampler::OptionFlags;
		quadloco::sim::Render const render
			( config
			, opt::None // no SceneBias nor ImageNoise
			);
		quadloco::ras::Grid<float> const gotPixGrid
			{ render.quadImage(numOverSample) };

		// For this test case, quad should exactly fill the 2x2 grid
		// E.g., gotPixGrid should contain values:
		//
		//     1.f  0.f
		//     0.f  1.f
		//

		// [DoxyExample00]

		if ( (2u == gotPixGrid.high())
		  && (2u == gotPixGrid.wide())
		   )
		{
			quadloco::ras::Grid<float> expPixGrid{ gotPixGrid.hwSize() };
			expPixGrid(0u, 0u) = 1.f;  expPixGrid(0u, 1u) = 0.f;  
			expPixGrid(1u, 0u) = 0.f;  expPixGrid(1u, 1u) = 1.f;  

			bool const samePixGrid
				{ std::equal
					( gotPixGrid.cbegin(), gotPixGrid.cend()
					, expPixGrid.cbegin()
					)
				};
			if (! samePixGrid)
			{
				oss << "Failure of exact target render test(1)\n";
				oss << expPixGrid.infoStringContents
					("# expPixGrid:", "%5.3f") << '\n';
				oss << gotPixGrid.infoStringContents
					("# gotPixGrid:", "%5.3f") << '\n';
			}
		}

	}

	//! Example general use
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		// simulation test configuration
		constexpr double edgeMag{ .125 };
		constexpr std::size_t numPix{ 128u };
		constexpr std::size_t numOverSample{ 64u }; // 0-> no over sampling

		// define a quad target object
		quadloco::obj::QuadTarget const objQuad
			( edgeMag
			, quadloco::obj::QuadTarget::None
		//	| quadloco::obj::QuadTarget::WithTriangle
			| quadloco::obj::QuadTarget::WithSurround
			);

		// simulate image of a quad target

		// configure camera and orientation such that
		// camera format exactly matches the objQuad target
		quadloco::sim::Config const config
			{ quadloco::sim::Config::faceOn(objQuad, numPix) };


		// render result
		using opt = quadloco::sim::Sampler::OptionFlags;
		quadloco::sim::Render const render
			( config
			, opt::None // no SceneBias nor ImageNoise
			);

		quadloco::ras::Grid<float> const fGrid{ render.quadImage() };
		// ... retrieve geometry of the simulated image
		quadloco::sig::QuadTarget const imgQuad{ render.sigQuadTarget() };

		// note min/max pixel values (e.g. useful for normalizing radiometry)
		quadloco::img::Span const fSpan
			{ quadloco::ras::grid::fullSpanFor(fGrid) };

		// [DoxyExample01]

		// Display results
		/*
		std::cout << "  fGrid: " << fGrid << '\n';
		std::cout << "  fSpan: " << fSpan << '\n';
		std::cout << "imgQuad: " << imgQuad << '\n';
		quadloco::io::writeStretchPGM("sample.pgm", fGrid);
		*/

		// TODO could used more sophisticated testing... but, for now
		// (since surround is also rendered)
		// just check that all rendered pixels are valid
		bool allValid{ true };
		std::for_each
			( fGrid.cbegin(), fGrid.cend()
			, [&allValid] (float const & pixVal)
				{ return engabra::g3::isValid(pixVal); }
			);
		if (! allValid)
		{
			oss << "Failure of all pixel valid test\n";
			oss << "  fGrid: " << fGrid << '\n';
			oss << "  fSpan: " << fSpan << '\n';
			oss << "imgQuad: " << imgQuad << '\n';
		}

	}

}

//! Check behavior of TODO
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

