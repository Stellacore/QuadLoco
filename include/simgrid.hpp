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


#include "rasGrid.hpp"
#include "rasRowCol.hpp"
#include "rasSizeHW.hpp"
#include "imgEdgel.hpp"


namespace quadloco
{

namespace sim
{

	//! Grid with a strong edge defined by provided edgel
	inline
	ras::Grid<float>
	gridWithEdge
		( ras::SizeHW const & hwSize
			//!< Size of grid to create
		, img::Edgel const & edgel
			//!< Location and direction of edge to create
		, float const & valueBackground = 0.f
			//!< Value to assign behind the edge
		, float const & valueForeground = 1.f
			//!< Value to assign in front of the edge
		)
	{
		ras::Grid<float> pixGrid(hwSize);
		for (std::size_t row{0u} ; row < hwSize.high() ; ++row)
		{
			for (std::size_t col{0u} ; col < hwSize.wide() ; ++col)
			{
				constexpr bool smoothEdge{ true };
				if (smoothEdge)
				{
					float const mag{ magnitude(edgel.gradient()) };
					constexpr float eps
						{ std::numeric_limits<float>::epsilon() };
					if ((256.f * eps) < mag)
					{
						pix::Spot const pixSpot{ (float)row, (float)col };
						pix::Spot const relSpot{ pixSpot - edgel.location() };
						float const delta{ dot(edgel.gradient(), relSpot) };
						float const dist{ (1.f/mag) * delta };
						float const mean
							{ .5f * (valueBackground + valueForeground) };
						float pixValue { mean + .5f*dist };
						if (pixValue < valueBackground)
						{
							pixValue = valueBackground;
						}
						if (! (pixValue < valueForeground))
						{
							pixValue = valueForeground;
						}
						pixGrid(row, col) = pixValue;
					}
				}
				else // sharp edge
				{
					ras::RowCol const rcLoc{ row, col };
					float pixValue{ valueBackground };
					if (edgel.rcInFront(rcLoc))
					{
						pixValue = valueForeground;
					}
					pixGrid(row, col) = pixValue;
				}
			}
		}
		return std::move(pixGrid);
	}

} // [sim]

} // [quadloco]

