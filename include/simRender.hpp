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
#include "simConfig.hpp"
#include "simSampler.hpp"

#include <Rigibra>


namespace quadloco
{

namespace sim
{
	//! Functor for rendering simulated quadrant images
	class Render
	{
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
			: theSampler(camera, xCamWrtQuad, objQuad, samplerOptions)
		{ }

		//! Construct rendering engine to simulate quad target images
		inline
		explicit
		Render
			( sim::Config const & simConfig
				//!< Simulation configuration
			, unsigned const & samplerOptions =
				( Sampler::UseSceneBias
				| Sampler::UseImageNoise
				)
				//!< XOR of quadloco::sim::Sampler::OptionFlags
			)
			: Render
				( simConfig.camera()
				, simConfig.xformStaWrtQuad()
				, simConfig.objQuadTarget()
				, samplerOptions
				)
		{ }

		//! True if this instance contains valid data
		inline
		bool
		isValid
			() const
		{
			return (theSampler.isValid());
		}

		//! Geometry of perspective image created by quadImage()
		inline
		img::QuadTarget
		imgQuadTarget
			() const
		{
			return theSampler.imgQuadTarget();
		}

		//! Simulate image through ctor's camera and orientation
		inline
		dat::Grid<float>
		quadImage
			( std::size_t const & numOverSamp = 64u
				//!< Number of *ADDITIONAL* intra-pixel *OVER* samplings
			) const
		{
			dat::Grid<float> grid(theSampler.format());
			std::fill(grid.begin(), grid.end(), engabra::g3::null<float>());
			injectTargetInto(&grid, numOverSamp);
			return grid;
		}

		//! Simulate image and set pixel values inside of provided grid
		inline
		void
		injectTargetInto
			( dat::Grid<float> * const & ptGrid
				//!< Destination into which to render image
			, std::size_t const & numOverSamp
				//!< Number of *ADDITIONAL* intra-pixel *OVER* samplings
			) const
		{
			dat::Grid<float> & grid = *ptGrid;

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
						{ (float)theSampler.intensityAt(detSpot, numOverSamp) };
					if (engabra::g3::isValid(inten))
					{
						grid(row, col) = inten;
					}
				}
			}
		}

		//! Descriptive information about this instance.
		inline
		std::string
		infoString
			( std::string const & title = {}
			) const
		{
			std::ostringstream oss;
			if (! title.empty())
			{
				oss << title << ' ';
			}
			oss
				<< "theSampler:\n" << theSampler
				;

			return oss.str();
		}

	}; // Render

} // [sim]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::sim::Render const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::sim::Render const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

