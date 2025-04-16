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
\brief Simulate a quad target image and save to file
*/


#include "QuadLoco/io.hpp"
#include "QuadLoco/objQuadTarget.hpp"
#include "QuadLoco/rasSizeHW.hpp"
#include "QuadLoco/simConfig.hpp"
#include "QuadLoco/simRender.hpp"

#include <iostream>


namespace quadloco
{
} // [quadloco]


/*! \brief Generate a simulated quad target image and save to file.
 *
 *
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
		std::cerr << "\nSimulate a quad target image and save to PGM file.";
		std::cerr << '\n';
		std::cerr << "Usage: <progname> <savePath.pgm>\n";
		std::cerr << '\n';
		std::cerr << 
			"  Program simulates a quad target image (using hard coded"
			"\nparameter values) and saved it to specified output path"
			"\n(in grayscale PGM format)."
			"\n\n"
			;
		return 1;
	}
	std::string const appName{ argv[0] };
	std::string const savePath{ argv[1] };

	if (! savePath.empty())
	{
		using namespace quadloco;

		// define camera geometry (ideal perspective) to use for simulation
		ras::SizeHW const format{ 80u, 120u }; // real cameras *much* larger
		double const pd{ 100. };
		obj::Camera const camera{ format, pd };

		// define a quad target object
		constexpr double edgeMag{ .050 }; // quad target 5cm on a side
		obj::QuadTarget const objQuad
			( edgeMag
			, quadloco::obj::QuadTarget::ConfigOptions
				{ .theWithTriangle = false
				, .theWithSurround = false
				}
			);

		// define camera location w.r.t. target [m]
		engabra::g3::Vector const camLoc{ .03, .05, .08 };

		// setup rendering object
		constexpr double rollSize{ .25 }; // .25 is near 14 deg
		sim::Render const render
			{ camera
			// xformCamWrtTgt - orients camera to point at target center
			, sim::Config::xformCamWrtTgt(camLoc, rollSize)
			, objQuad
			, quadloco::sim::Sampler::RenderOptions
				{ .theAddSceneBias = true
				, .theAddImageNoise = true
				}
			};

		// simulation test configuration
		constexpr std::size_t numOverSample{ 256u }; // 0-> no over sampling
		ras::Grid<float> const pixGrid{ render.quadGrid(numOverSample) };

		// save to file
		bool const okaySave{ quadloco::io::writeStretchPGM(savePath, pixGrid) };


		// summarize result
		std::cout <<
			"\nSummary:"
			"\nProgram: " << appName
			;
		if (okaySave)
		{
			std::cout << "\nSuccess:";
		}
		else
		{
			std::cout << "\nERROR - Unable to write output file!";
		}
		std::cout <<
			"\nOutFile: " << savePath << 
			"\n\n";

	}

	return 0;
}


