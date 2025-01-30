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
 * \brief Declarations for functions in quadloco::sim::grid namespace
 *
 */


#include "QuadLoco/cast.hpp"
#include "QuadLoco/imgEdgel.hpp"
#include "QuadLoco/imgSpot.hpp"
#include "QuadLoco/pix.hpp"
#include "QuadLoco/rasGrid.hpp"
#include "QuadLoco/rasRowCol.hpp"
#include "QuadLoco/rasSizeHW.hpp"


namespace quadloco
{

namespace sim
{
	//! Specify property of edge transition
	enum Transition
	{
		  Step //!< Abrupt transition from background to foreground value
		, Ramp //!< Linear interpolation at scale of grid cell distance
	};

	//! Grid with a strong edge defined by provided edgel
	inline
	ras::Grid<float>
	gridWithEdge
		( ras::SizeHW const & hwSize
			//!< Size of grid to create
		, img::Edgel const & edgel
			//!< Location and direction of edge to create
		, Transition const & edgeDetail = Ramp
			//!< Specify edge characteristic
		, float const & valueBackground = 0.
			//!< Value to assign behind the edge
		)
	{
		ras::Grid<float> pixGrid(hwSize);
		if (! edgel.isValid())
		{
			// if bad input data, 
			std::fill(pixGrid.begin(), pixGrid.end(), pix::fNull);
		}
		else
		{
			// determine forground value (i.e. after the edgel gradient)
			constexpr double rcDelta{ 1. };
			double const gradMag{ magnitude(edgel.gradient()) };
			double const valueDelta{ rcDelta * gradMag };
			float const valueForeground{ valueBackground + (float)valueDelta };
			float const valueMean{ .5f * (valueBackground + valueForeground) };

			// set each pixel value in return grid
			for (std::size_t row{0u} ; row < hwSize.high() ; ++row)
			{
				for (std::size_t col{0u} ; col < hwSize.wide() ; ++col)
				{
					double pixValue{ pix::fNull };
					if (Ramp == edgeDetail)
					{
						// linear interpolation at edge
						img::Spot const imgSpot{ (double)row, (double)col };
						img::Spot const relSpot{ imgSpot - edgel.location() };
						double const dist{ dot(edgel.direction(), relSpot) };

						// start w/ linear interpolation of value from edge...
						pixValue = (valueMean + (float)(valueDelta * dist));
						// ... then clip at background or foreground values
						if (pixValue < valueBackground)
						{
							pixValue = valueBackground;
						}
						if (! (pixValue < valueForeground))
						{
							pixValue = valueForeground;
						}
					}
					else
					if (Step == edgeDetail)
					{
						// immediate step up at edge
						img::Spot const spotLoc{ row, col };
						if (edgel.spotInFront(spotLoc))
						{
							pixValue = valueForeground;
						}
						else
						{
							pixValue = valueBackground;
						}
					}
					pixGrid(row, col) = pixValue;
				}
			}
		}
		return pixGrid;
	}

} // [sim]

} // [quadloco]

