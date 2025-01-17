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
 * \brief Declarations for quadloco::ops::CenterRefinerEdge namespace
 *
 */


#include "imgGrad.hpp"
#include "imgHit.hpp"
#include "imgSpot.hpp"
#include "opsgrid.hpp"
#include "rasGrid.hpp"
#include "rasPeakRCV.hpp"


namespace quadloco
{

namespace ops
{

	/*! \brief Refine center locations with line fitting (to edge magnitudes).
	 *
	 * Upon construction, gradient values are computed for the entire
	 * source image. These values are used for subsequent center candidate
	 * refinements.
	 *
	 */
	class CenterRefinerEdge
	{
		//! Gradient values for each source image cell.
		ras::Grid<img::Grad> const theGradGrid{};

	public:

		//! Compute and cache gradient values for use in other methods.
		inline
		explicit
		CenterRefinerEdge
			( ras::Grid<float> const & srcGrid
			)
			: theGradGrid{ ops::grid::gradientGridBy8x(srcGrid) }
		{ }

		//! Compute refine center point hits for all peakRCVs candidates
		inline
		std::vector<img::Hit>
		centerHits
			( std::vector<ras::PeakRCV> const & // peakRCVs
			) const
		{
			return {};//TODO
		}


/*
 * The function fitSpotNear() computes a refined center point in the
 * vicinity of a candidate target center location. The refined location
 * is determined from the gradient values in the specified neighborhood
 * of the candidate location.
 */

		/*! \brief Estimated quad target center refinement based on edges.
		 */
		/*
		inline
		img::Spot
		fitSpotNear
			( ras::RowCol const & // rcHoodCenterInSrc
				//!< Center of window in which to search for edges
			, std::size_t const & // radiusSearch = 7u
				//!< Radius over which to consider edges
			) const
		{
			return img::Spot{}; //TODO
		}
		*/

	}; // CenterRefinerEdge


} // [ops]

} // [quadloco]

