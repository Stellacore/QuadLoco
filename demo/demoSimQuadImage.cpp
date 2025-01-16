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


#include "io.hpp"
#include "objQuadTarget.hpp"
#include "simConfig.hpp"
#include "simRender.hpp"

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
		std::cout <<
			"\nSimulate a quad target image and save to (PGM) file"
			"\n"
			"\nUsage: <progname> <outFileName>.pgm"
			"\n\n";
		return 1;
	}
	std::string const appName{ argv[0] };
	std::string const savePath{ argv[1] };

	// configuration parameters
	double const edgeMag{ .125 };
	std::size_t const numPix{ 64u };
	std::size_t const numOverSamp{ 256u };


	// define quad target
	using quadloco::obj::QuadTarget;
	QuadTarget const objQuad
		{ edgeMag
		, QuadTarget::None
		};

	// specify imaging geometry
	quadloco::sim::Config const config
		{ quadloco::sim::Config::faceOn(objQuad, numPix) };

	// render image
	quadloco::sim::Render const render
		{ config
		, quadloco::sim::Sampler::OptionFlags::AddImageNoise
		};
	quadloco::ras::Grid<float> const pixGrid
		{ render.quadImage(numOverSamp) };


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

	return 0;
}


