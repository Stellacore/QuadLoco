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
#include "objQuadTarget.hpp"
#include "simSampler.hpp"

#include <Rigibra>


namespace quadloco
{

namespace sim
{
	//! Functor for rendering simulated quadrant images
	struct Render
	{
		img::Camera const theCamera{};
		rigibra::Transform const theCamWrtQuad{};
		obj::QuadTarget const theObjQuad{};

		//! Simulate an image through camera at position xCamWrtQuad
		inline
		void
		injectTargetInto
			( dat::Grid<float> * const & ptGrid
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

			Sampler const sampler(theCamera, theCamWrtQuad, theObjQuad);

			using namespace rigibra;
			for (std::size_t row{0u} ; row < grid.high() ; ++row)
			{
				for (std::size_t col{0u} ; col < grid.wide() ; ++col)
				{
					double const subRow{ (double)row };
					double const subCol{ (double)col };
					dat::Spot const detSpot{ subRow, subCol };

					float const inten{ (float)sampler.intensityAt(detSpot) };
					if (engabra::g3::isValid(inten))
					{
						grid(row, col) = inten;
					}
				}
			}
		}

		//! Simulate an image through camera at position xCamWrtQuad
		inline
		dat::Grid<float>
		quadImage
			() const
		{
			dat::Grid<float> grid(theCamera.theFormat);
			std::fill(grid.begin(), grid.end(), engabra::g3::null<float>());
			injectTargetInto(&grid);
			return grid;
		}

	}; // Render

} // [sim]

} // [quadloco]

