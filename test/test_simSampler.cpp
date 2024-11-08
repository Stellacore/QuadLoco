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
\brief Unit tests (and example) code for quadloco::sim::Sampler
*/


#include "datGrid.hpp"
#include "io.hpp"
#include "simConfig.hpp"
#include "sim.hpp"
#include "simSampler.hpp"

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

		// setup simulation configuration
		constexpr quadloco::dat::SizeHW hwChip{ 60u, 40u };
		quadloco::obj::QuadTarget const objQuad(2.100);

		using namespace engabra::g3;
		using namespace rigibra;
		std::vector<Transform> const xStaWrtQuads
			{ Transform
				{ Vector{ .5, -.5, 1. }
				, Attitude{ PhysAngle{ BiVector{ .5, 0., 0. } } }
				}
			};
		for (rigibra::Transform const & xStaWrtQuad : xStaWrtQuads)
		{
/*
			quadloco::sim::Config const config
				{ quadloco::sim::Config::toViewQuadFrom
					(hwChip, xStaWrtQuad, objQuad)
				};

std::cout << '\n';
std::cout << "config:\n" << config << '\n';
std::cout << '\n';

			quadloco::dat::Grid<float> const fGrid
				{ quadloco::sim::quadImage
					(config.theCamera, config.theStaWrtQuad, objQuad)
				};

quadloco::io::writeStretchPGM("sample.pgm", fGrid);
*/

		}

	/*
		using engabra::g3::Vector;

		quadloco::sim::Config const config

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
	*/

		// [DoxyExample01]

		// TODO replace this with real test code
		std::string const fname(__FILE__);
		bool const isTemplate{ (std::string::npos != fname.find("/_.cpp")) };
		if (! isTemplate)
		{
			oss << "Failure to implement real test\n";
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

