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
 * \brief Declarations for quadloco::sig::edgel namespace
 *
 */


#include "imgEdgel.hpp"
#include "imgGrad.hpp"
#include "opsgrid.hpp"
#include "rasGrid.hpp"
#include "sigEdgeInfo.hpp"

#include <algorithm>
#include <vector>


namespace quadloco
{

namespace sig
{

//! \brief Functions and utilities for working with edgel collections
namespace edgel
{
	/*! \brief Multiplication threshold for neighborhood gradient agreement.
	 *
	 * This value is used to compare gradient values in neighboring cells.
	 * Compute the sum of the 8-neighbor gradients projected onto the
	 * gradient direction at the neighborhood center. If the ratio of
	 * neighbor projections compared with center gradient magnitude is
	 * this large (or more), then consider the neighborhood center gradient
	 * to be "corroborated" by neighbors.
	 */
	constexpr double sHoodSupportRatio{ 2.50 };

	/*! \brief Limit number of "strong" edgels values considered for geometry.
	 *
	 * In an extreme case, the target radial edges might be aligned with
	 * the image chip diagonal. If each edge is two pixels wide, the
	 * radial edgs would account for 4*diag number of pixels. For triangle
	 * clipped targets, the triangle bases could each have multiple pixel
	 * wide edges - so add another 2*diag for that.
	 */
	constexpr std::size_t sDiagSizeMultiple{ 6u };

	//! Extract the largest magnitude edgels from grid
	inline
	std::vector<img::Edgel>
	dominantEdgelsFrom
		( ras::Grid<img::Grad> const & gradGrid
		, double const & hoodSupportRatio = sHoodSupportRatio
		, std::size_t const & diagSizeMultiple = sDiagSizeMultiple
		)
	{
		// get all strongly linked edgels
		std::vector<img::Edgel> edgels
			{ ops::grid::linkedEdgelsFrom(gradGrid, hoodSupportRatio) };

		// conservative estimate of how many to search knowing that
		// a quad target should be consuming large part of grid
		double const diag{ gradGrid.hwSize().diagonal() };
		std::size_t const estNumToUse
			{ static_cast<std::size_t>((double)diagSizeMultiple * diag) };
		std::size_t const numToUse{ std::min(estNumToUse, edgels.size()) };

		// move strongest edges near front
		std::partial_sort
			( edgels.begin(), edgels.begin()+numToUse
			, edgels.end()
			, [] (img::Edgel const & e1, img::Edgel const & e2)
				// sort in *desending* order
				{ return (e2.magnitude() < e1.magnitude()); }
			);

		// ignore less significant edges in return structure
		edgels.resize(numToUse);
		return edgels;
	}

	//! Determine (*combinatorially*) edgel radial edge (pseudo)likelihood
	inline
	std::vector<sig::EdgeInfo>
	edgeInfosLikelyRadial
		( std::vector<img::Edgel> const & edgels
		)
	{
		// Individual edge properties
		std::vector<sig::EdgeInfo> edgeInfos;
		std::size_t const numElem{ edgels.size() };
		if (2u < numElem)
		{
			edgeInfos.reserve(numElem);

			// compute useful edgel properties suggesting
			// edgels likely to be on opposite radial edges
			for (std::size_t ndx1{0u} ; ndx1 < numElem ; ++ndx1)
			{
				// construct EdgeInfo tracking instance
				img::Edgel const & edgel = edgels[ndx1];
				edgeInfos.emplace_back(sig::EdgeInfo(edgel));

				// update just inserted edgel with prior edgels (below)
				sig::EdgeInfo & edgeInfo1 = edgeInfos.back();

				// compare from begin thru just *before* last item added
				std::size_t const numPrev{ edgeInfos.size() - 1u };
				for (std::size_t ndx2{0u} ; ndx2 < numPrev ; ++ndx2)
				{
					// combinatorially consider other edges and
					// update each with consideration of the other
					sig::EdgeInfo & edgeInfo2 = edgeInfos[ndx2];
					edgeInfo1.considerOther(edgeInfo2.edgel());
					edgeInfo2.considerOther(edgeInfo1.edgel());
				}
			}
		}

		// filter very low weight edges
// TODO - determine what is a low weight
		return edgeInfos;
	}


}; // [edgel]


} // [sig]

} // [quadloco]

