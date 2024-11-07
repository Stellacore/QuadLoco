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


#include "datGrid.hpp"
#include "io.hpp"
#include "objQuadTarget.hpp"
#include "pix.hpp"
#include "sim.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>


namespace tst
{
	using namespace engabra::g3;
	using namespace rigibra;

	//! Test case camera view to use for rendering in simulation
	struct CamOri
	{
		//! Camera for rendering target
		quadloco::img::Camera const theCamera
			{ quadloco::dat::SizeHW{ 24u, 24u }  // format
			, double{ 175. }// principal distance
			};

		//! Camera exterior orientation
		Transform const theCamWrtQua
			{ Vector{ .47, -.32, 1. } // above target
			, Attitude{ PhysAngle{ BiVector{ .3, .4, .1} } }
			};

	}; // CamOri

} // [tst]

namespace
{
	//! Examples for documentation
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		// define a quad target object
		constexpr double edgeMag{ .125 };
		quadloco::obj::QuadTarget const objQuad(edgeMag, true);

		// simulate image of a quad target

		// an oriented ideal perspective camera
		tst::CamOri const camOri{};

		quadloco::dat::Grid<float> const fGrid
			{ quadloco::sim::quadImage
				(camOri.theCamera, camOri.theCamWrtQua, objQuad)
			};

quadloco::io::writeStretchPGM("sample.pgm", fGrid);

		// detect quad parameters in (perspective) image
//		quadloco::img::QuadTarget const imgQuad
//			{ sim::imageQuadFor(fGrid) };

		// assess the "quadness" of a pixel sampling
//		double const gotQuadness{ quadloco::quadnessOf(fGrid, imgQuad) };

		// [DoxyExample01]


quadloco::dat::Span const fSpan{ quadloco::pix::fullSpanFor(fGrid) };
quadloco::dat::Grid<uint8_t> const uGrid
	{ quadloco::pix::uGrid8(fGrid, fSpan) };
std::ofstream ofs("/dev/stdout");
ofs << '\n';
ofs << fGrid.infoStringContents("fGrid:\n", "%4.2f") << '\n';
ofs << uGrid.infoStringContents("uGrid:\n", "%4u") << '\n';
ofs << '\n';

		// TODO replace this with real test code
		std::string const fname(__FILE__);
		bool const isTemplate{ (std::string::npos != fname.find("/_.cpp")) };
		if (! isTemplate)
		{
			oss << "Failure to implement real test\n";
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

