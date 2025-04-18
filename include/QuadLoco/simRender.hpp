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


#include "QuadLoco/cast.hpp"
#include "QuadLoco/imgQuadTarget.hpp"
#include "QuadLoco/objCamera.hpp"
#include "QuadLoco/objQuadTarget.hpp"
#include "QuadLoco/rasChipSpec.hpp"
#include "QuadLoco/rasGrid.hpp"
#include "QuadLoco/simConfig.hpp"
#include "QuadLoco/simSampler.hpp"

#include <Rigibra>


namespace quadloco
{

namespace sim
{
	//! \brief Simulated target data (grid and image geometry)
	struct QuadData
	{
		ras::Grid<float> const theGrid;
		img::QuadTarget const theImgQuad;

	}; // QuadData


	//! Functor for rendering simulated quadrant images
	class Render
	{
		//! Cached data - must be set in ctor
		Sampler const theSampler;

	public: // static functions

		//! Simulated grid and image geometry data.
		inline
		static
		QuadData
		simpleQuadData
			( std::size_t const & formatAndPd = 32u
			, std::size_t const & numOverSample = 64u
			, engabra::g3::Vector const & dirToCamera = { 0., 0., 1. }
			, engabra::g3::BiVector const & physAngleCamWrtTgt = { 0., 0., 0. }
			, double const & objQuadEdgeSize = 1.
			)
		{
			using namespace engabra::g3;
			using namespace rigibra;
			using quadloco::obj::QuadTarget;
			using quadloco::sim::Sampler;

			sim::Render const render
				( obj::Camera::isoCam(formatAndPd)
				, Transform
					{ dirToCamera
					, Attitude{ PhysAngle{ physAngleCamWrtTgt } }
					}
				, QuadTarget
					( objQuadEdgeSize
					, QuadTarget::ConfigOptions
						{ .theWithTriangle = true
						, .theWithSurround = true
						}
					)
				, Sampler::RenderOptions
					{ .theAddSceneBias = true
					, .theAddImageNoise = true
					}
				);

			return render.quadData(numOverSample);
		}

	public:

		//! Construct rendering engine to simulate quad target images
		inline
		explicit
		Render
			( obj::Camera const & camera
				//!< Camera geometry with which to render image
			, rigibra::Transform const & xCamWrtQuad
				//!< Orientation of camera (exterior) frame w.r.t. objQuad
			, obj::QuadTarget const & objQuad
				//!< Quad target (defines the quad reference frame) 
			, Sampler::RenderOptions const & renderOptions =
				{ .theAddSceneBias = true
				, .theAddImageNoise = true
				}
				//!< XOR of quadloco::sim::Sampler::OptionFlags
			)
			: theSampler(camera, xCamWrtQuad, objQuad, renderOptions)
		{ }

		//! Construct rendering engine to simulate quad target images
		inline
		explicit
		Render
			( sim::Config const & simConfig
				//!< Simulation configuration
			, Sampler::RenderOptions const & renderOptions =
				{ .theAddSceneBias = true
				, .theAddImageNoise = true
				}
				//!< XOR of quadloco::sim::Sampler::OptionFlags
			)
			: Render
				( simConfig.camera()
				, simConfig.xformStaWrtQuad()
				, simConfig.objQuadTarget()
				, renderOptions
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

		//! Geometry of perspective image created by quadGrid()
		inline
		img::QuadTarget
		imgQuadTarget
			() const
		{
			return theSampler.imgQuadTarget();
		}

		//! Simulate chip of an image through ctor's camera and orientation
		inline
		ras::Grid<float>
		quadChip
			( std::size_t const & numOverSamp = 64u
				//!< Number of *ADDITIONAL* intra-pixel *OVER* samplings
			, ras::ChipSpec const & chipSpec = {}
				//!< Define chip patch area to render (default is full grid)
			) const
		{
			ras::Grid<float> chip;
			if (chipSpec.isValid())
			{
				chip = ras::Grid<float>(chipSpec.hwSize());
				std::fill(chip.begin(), chip.end(), engabra::g3::null<float>());
				injectTargetInto(&chip, numOverSamp, chipSpec);
			}
			return chip;
		}

		//! Simulate image through ctor's camera and orientation
		inline
		ras::Grid<float>
		quadGrid
			( std::size_t const & numOverSamp = 64u
				//!< Number of *ADDITIONAL* intra-pixel *OVER* samplings
			) const
		{
			ras::Grid<float> grid(theSampler.format());
			std::fill(grid.begin(), grid.end(), engabra::g3::null<float>());
			injectTargetInto(&grid, numOverSamp);
			return grid;
		}

		//! Simulated grid and image geometry data.
		inline
		QuadData
		quadData
			( std::size_t const & numOverSamp = 64u
				//!< Number of *ADDITIONAL* intra-pixel *OVER* samplings
			) const
		{
			return QuadData
				{ quadGrid(numOverSamp)
				, imgQuadTarget()
				};
		}

		//! Simulate image and set pixel values inside of provided grid
		inline
		void
		injectTargetInto
			( ras::Grid<float> * const & ptGrid
				//!< Destination into which to render image
			, std::size_t const & numOverSamp
				//!< Number of *ADDITIONAL* intra-pixel *OVER* samplings
			, ras::ChipSpec const & chipSpec = {}
				//!< Define chip patch area to render (default is full grid)
			) const
		{
			ras::Grid<float> & grid = *ptGrid;

			if (ptGrid && ptGrid->isValid())
			{
				// define rendering area
				ras::RowCol rc0{ 0u, 0u };
				ras::SizeHW hwSize{ ptGrid->hwSize() };
				if (chipSpec.isValid())
				{
					rc0 = chipSpec.srcOrigRC();
					hwSize = chipSpec.hwSize();
				}
				ras::ChipSpec chip{ rc0, hwSize };

				for (std::size_t rowChip{0u} ; rowChip < chip.high()
					; ++rowChip)
				{
					for (std::size_t colChip{0u} ; colChip < chip.wide()
						; ++colChip)
					{
						ras::RowCol const rcChip{ rowChip, colChip };
						ras::RowCol const rcGrid
							{ chip.rcFullForChipRC(rcChip) };

						// location in full camera format
						img::Spot const detSpot{ cast::imgSpot(rcGrid) };

						// intensity at grid location
						float const inten
							{ (float)theSampler.intensityAt
								(detSpot, numOverSamp)
							};
						if (engabra::g3::isValid(inten))
						{
							grid(rcChip) = inten;
						}
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

