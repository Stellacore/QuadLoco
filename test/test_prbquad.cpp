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
\brief Unit tests (and example) code for quadloco::TODO
*/


#include "prbquad.hpp"

#include "objCamera.hpp"
#include "objQuadTarget.hpp"
#include "rasSizeHW.hpp"
#include "sigQuadTarget.hpp"
#include "simConfig.hpp"
#include "simRender.hpp"

#include <Engabra>
#include <Rigibra>

#include <iostream>
#include <sstream>


namespace
{
	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		// define a quad target object
		constexpr double edgeMag{ .125 };
		constexpr std::size_t numPix{ 8u };
		constexpr std::size_t numOver{ 0u };
		quadloco::obj::QuadTarget const objQuad
			( edgeMag
			, quadloco::obj::QuadTarget::None
		//	| quadloco::obj::QuadTarget::WithTriangle
		//	| quadloco::obj::QuadTarget::WithSurround
			);

		// simulate face-on 1:1 image of a quad target
		quadloco::sim::Config const config
			{ quadloco::sim::Config::faceOn(objQuad, numPix) };
		quadloco::obj::Camera const & camera = config.camera();
		rigibra::Transform const & xCamWrtQua = config.xformStaWrtQuad();

		// render simulated image
		using opt = quadloco::sim::Sampler::OptionFlags;
		quadloco::sim::Render const render
			( config
		//	( camera, xCamWrtQua, objQuad
			, opt::None // no SceneBias nor ImageNoise
			);
		quadloco::ras::Grid<float> const pixGrid{ render.quadImage(numOver) };
		// retrieve geometry of the simulated image
		quadloco::sig::QuadTarget const expImgQuad{ render.imgQuadTarget() };

		// for this test, assume the found geometry is perfect
		quadloco::sig::QuadTarget const & gotImgQuad = expImgQuad;

		// assess the quadness of pixels w.r.t. the estimated "found" geometry
		std::ostringstream msg; // diagnostic info
		double const probQuad
			{ quadloco::prb::isQuadlike(pixGrid, gotImgQuad, &msg) };

		// [DoxyExample01]

		// check if the result is more likely than not to be "quad-like"
		bool const isQuad{ .5 < probQuad };
		if (! isQuad)
		{
			oss << "Failure of isQuad test\n";
			oss << pixGrid.infoStringContents("pixGrid", "%5.2f") << '\n';
			oss << "expImgQuad: " << expImgQuad << '\n';
			oss << "probQuad: " << probQuad << '\n';
			oss << "msg: " << msg.str() << '\n';
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

