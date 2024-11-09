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


#pragma once


/*! \file
 * \brief Declarations for quadloco::sim::Render
 *
 */


#include "datGrid.hpp"
#include "imgCamera.hpp"
#include "imgQuadTarget.hpp"
#include "objQuadTarget.hpp"
#include "simSampler.hpp"

#include <Rigibra>


namespace quadloco
{

namespace sim
{
	//! Functor for rendering simulated quadrant images
	class Render
	{
		//! Camera with which to render image geometry and intensities
		img::Camera const theCamera{};

		//! Exterior orientation of camera w.r.t. quad target
		rigibra::Transform const theCamWrtQuad{};

		//! Quad target geometry in object space
		obj::QuadTarget const theObjQuad{};

		//! Cached data - must be set in ctor
		Sampler const theSampler;

	public:

		//! Construct rendering engine to simulate quad target images
		inline
		explicit
		Render
			( img::Camera const & camera
				//!< Camera geometry with which to render image
			, rigibra::Transform const & xCamWrtQuad
				//!< Orientation of camera (exterior) frame w.r.t. objQuad
			, obj::QuadTarget const & objQuad
				//!< Quad target (defines the quad reference frame) 
			, unsigned const & samplerOptions =
				( Sampler::UseSceneBias
				| Sampler::UseImageNoise
				)
				//!< XOR of quadloco::sim::Sampler::OptionFlags
			)
			: theCamera{ camera }
			, theCamWrtQuad{ xCamWrtQuad }
			, theObjQuad{ objQuad }
			, theSampler(theCamera, theCamWrtQuad, theObjQuad, samplerOptions)
		{ }

		//! Geometry of perspective image created by quadImage()
		inline
		img::QuadTarget
		imgQuadTarget
			() const
		{
			return theSampler.imgQuadTarget();
		}

		//! Simulate an image through camera at position xCamWrtQuad
		inline
		dat::Grid<float>
		quadImage
			( std::size_t const & numSubSamp = 64u
			) const
		{
			dat::Grid<float> grid(theCamera.theFormat);
			std::fill(grid.begin(), grid.end(), engabra::g3::null<float>());
			injectTargetInto(&grid, numSubSamp);
			return grid;
		}

		//! Simulate an image through camera at position xCamWrtQuad
		inline
		void
		injectTargetInto
			( dat::Grid<float> * const & ptGrid
			, std::size_t const & numSubSamp
			) const
		{
			dat::Grid<float> & grid = *ptGrid;
			constexpr double objDelta{ 1./1024. };

/*
double const & beg0 = theObjQuad.span0().theBeg;
double const & end0 = theObjQuad.span0().theEnd;
double const & beg1 = theObjQuad.span1().theBeg;
double const & end1 = theObjQuad.span1().theEnd;
*/

			using namespace rigibra;
			for (std::size_t row{0u} ; row < grid.high() ; ++row)
			{
				for (std::size_t col{0u} ; col < grid.wide() ; ++col)
				{
					double const subRow{ (double)row };
					double const subCol{ (double)col };
					dat::Spot const detSpot{ subRow, subCol };

					float const inten
						{ (float)theSampler.intensityAt(detSpot, numSubSamp) };
					if (engabra::g3::isValid(inten))
					{
						grid(row, col) = inten;
					}
				}
			}
		}

	}; // Render

} // [sim]

} // [quadloco]

