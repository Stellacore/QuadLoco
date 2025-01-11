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


#include "appQuadLike.hpp"
#include "io.hpp"
#include "rasGrid.hpp"
#include "simConfig.hpp"
#include "sim.hpp"
#include "simRender.hpp"
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
		quadloco::obj::QuadTarget const objQuad(2.100);

		quadloco::ras::SizeHW const format{ 128u, 128u };
		double const pd{ 128. };
		quadloco::obj::Camera const camera{ format, pd };

		using namespace engabra::g3;
		using namespace rigibra;
		struct TestCase
		{
			Transform theStaWrtQuad{};
			double theQuadProb{ engabra::g3::null<double>() };
		};

		std::vector<TestCase> testcases
			{ TestCase
				{ Transform
					{ Vector{ .0,  .0, 1. }
					, Attitude{ PhysAngle{ BiVector{ .0, 0., 0. } } }
					}
				}
			, TestCase
				{ Transform
					{ Vector{ .5, -.5, 1. }
					, Attitude{ PhysAngle{ BiVector{ .5, 0., 0. } } }
					}
				}
			, TestCase
				{ Transform
					{ Vector{ .0,  .0, 1. }
					, Attitude{ PhysAngle{ BiVector{ .0, 0., 1.25 } } }
					}
				}
			};

		// run multiple test cases
		for (TestCase & testcase : testcases)
		{
			quadloco::sim::Config const config
				{ objQuad
				, camera
				, testcase.theStaWrtQuad
				};

			// render result
			using opt = quadloco::sim::Sampler::OptionFlags;
			quadloco::sim::Render const render
				( config
				// None, AddSceneBias, AddImageNoise
				, opt::None | opt::AddImageNoise
				);
			quadloco::ras::Grid<float> const pixGrid{ render.quadImage() };
			//quadloco::io::writeStretchPGM("sample.pgm", pixGrid);

			// retrieve geometry of the simulated image
			quadloco::img::QuadTarget const expImgQuad
				{ render.imgQuadTarget() };
			std::ostringstream msg;
			double const quadProb
				{ quadloco::app::isQuadlike(pixGrid, expImgQuad, &msg) };

			// note test case result
			testcase.theQuadProb = quadProb;

		}

		// [DoxyExample01]

		// check that all quads are more likely than not
		bool probAll{ true };
		for (TestCase const & testcase : testcases)
		{
			probAll &= (.5 < testcase.theQuadProb);
		}

		if (! probAll)
		{
			oss << "Failure of all probability test\n";
			for (TestCase const & testcase : testcases)
			{
				double const & quadProb = testcase.theQuadProb;
				oss << "quadProb:\n" << std::fixed << quadProb << '\n';
			}
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

