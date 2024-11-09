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
#include "simRender.hpp"

#include <algorithm>
#include <fstream>
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

		// define a quad target object
		constexpr double edgeMag{ .125 };
		quadloco::obj::QuadTarget const objQuad
			( edgeMag
			, quadloco::obj::QuadTarget::None
		//	| quadloco::obj::QuadTarget::DoubleTriangle
			| quadloco::obj::QuadTarget::AddSurround
			);

		// simulate image of a quad target

		// configure camera to produce near identity rendering
		quadloco::img::Camera const camera
			{ quadloco::dat::SizeHW{ 128u, 128u }  // format
			, double{ 128. }// principal distance (== to quad edge)
			};
		// exterior orientation directly above the quad target
		double const camOriZ{ 1.125*edgeMag }; // get a bit of the surround
		rigibra::Transform const xCamWrtQua
			{ engabra::g3::Vector{ 0., 0., camOriZ }
			, rigibra::identity<rigibra::Attitude>()
			};

		// render simulated image and ...
		quadloco::sim::Render const render(camera, xCamWrtQua, objQuad);
		quadloco::dat::Grid<float> const fGrid{ render.quadImage() };
		// ... retrieve geometry of the simulated image
		quadloco::img::QuadTarget const imgQuad{ render.imgQuadTarget() };

std::cout << "  fGrid: " << fGrid << '\n';
std::cout << "imgQuad: " << imgQuad << '\n';

quadloco::io::writeStretchPGM("sample.pgm", fGrid);

		// [DoxyExample01]


/*
quadloco::dat::Span const fSpan{ quadloco::pix::fullSpanFor(fGrid) };
quadloco::dat::Grid<uint8_t> const uGrid
	{ quadloco::pix::uGrid8(fGrid, fSpan) };
std::ofstream ofs("/dev/stdout");
ofs << '\n';
ofs << uGrid.infoStringContents("uGrid:\n", "%4u") << '\n';
ofs << fGrid.infoStringContents("fGrid:\n", "%4.2f") << '\n';
ofs << '\n';
*/

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

