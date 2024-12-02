
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
 * \brief Top level file for quadloco::sig::EdgeEval
 *
 */


#include "angLikely.hpp"
#include "opsgrid.hpp"
#include "rasGrid.hpp"
#include "sigEdgeInfo.hpp"

#include <algorithm>
#include <vector>


namespace quadloco
{

namespace sig
{

	//! \brief Evaluator of edgels that are likely part of quad target image
	class EdgeEval
	{
		std::vector<img::Edgel> const theEdgels;
		std::vector<sig::EdgeInfo> const theEdgeInfos;

	// static functions

		//! Extract the largest magnitude edgels from grid
		inline
		static
		std::vector<img::Edgel>
		dominantEdgelsFrom
			( ras::Grid<img::Grad> const & gradGrid
			, std::size_t const & numTimesDiagonal = 6u
			)
		{
			// get all strongly linked edgels
			std::vector<img::Edgel> edgels
				{ ops::grid::linkedEdgelsFrom(gradGrid, 1.5) };

			// conservative estimate of how many to search knowing that
			// a quad target should be consuming large part of grid
			double const diag{ gradGrid.hwSize().diagonal() };
			std::size_t const estNumToUse
				{ static_cast<std::size_t>((double)numTimesDiagonal * diag) };
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
		static
		std::vector<sig::EdgeInfo>
		edgeInfosFor
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
					sig::EdgeInfo edgeInfo1(edgel);
					edgeInfos.emplace_back(edgeInfo1);

					// compare from start until *before* last item just added
					std::size_t const numLast{ edgeInfos.size() - 1u };
					for (std::size_t ndx2{0u} ; ndx2 < numLast ; ++ndx2)
					{
						// combinatorially consider other edges and
						// update each with consideration of the other
						sig::EdgeInfo & edgeInfo2 = edgeInfos[ndx2];
						edgeInfo1.consider(edgeInfo2.edgel());
						edgeInfo2.consider(edgeInfo1.edgel());
					}
				}
			}
			return edgeInfos;
		}

	public:

		// Process gradients into non-isolated edges
		inline
		explicit
		EdgeEval
			( ras::Grid<img::Grad> const & gradGrid
			)
// TODO - really don't need to save edgels, since in they are in EdgeInfos
			: theEdgels{ dominantEdgelsFrom(gradGrid) }
			, theEdgeInfos{ edgeInfosFor(theEdgels) }
		{ }

		//! Determine most likely edgel angle directions (perp to radial edges)
		inline
		std::vector<double>
		peakAngles
			( std::size_t const & numAngBins = 32u
			) const
		{
			std::vector<double> peaks;

std::cout << "\npeakAngles():\n";
std::cout << "theEdgeInfos.size: " << theEdgeInfos.size() << '\n';
			// create angle value accumulation (circular-wrapping) buffer
			ang::Likely angleProbs(numAngBins);
			for (sig::EdgeInfo const & edgeInfo : theEdgeInfos)
			{
				double const fwdAngle{ edgeInfo.consideredAngle() };
				double const weight{ edgeInfo.consideredWeight() };
				angleProbs.add(fwdAngle, weight, 2u);
				// TODO - is this necessary?
				// also add opposite angle to ensure angle prob buffer
				// peaks are symmetrically balanced (to faciliate 
				// edge group duo extraction below
//				double const revAngle
//					{ ang::principalAngle(fwdAngle + ang::piOne()) };
//				angleProbs.add(revAngle, weight, 2u);
			}

std::cout << angleProbs.infoString("angleProbs") << '\n';
std::cout << angleProbs.infoStringContents("angleProbs") << '\n';

			// get peaks from angular accumulation buffer
			peaks = angleProbs.anglesOfPeaks();

for (double const & peakAngle : peaks)
{
	double const peakValue{ angleProbs.binSumAtAngle(peakAngle) };
	std::cout
		<< "  peakAngle: " << engabra::g3::io::fixed(peakAngle)
		<< "  peakValue: " << engabra::g3::io::fixed(peakValue)
		<< '\n';
}

			return peaks;
		}

		/*
		//! Collection of EdgeInfo likely belonging to same radial edge
		using EdgeGroup = std::vector<sig::EdgeInfo>;

		//! (Radial) edge groups on opposite side of target center
		struct EdgeGroupDuo
		{
			EdgeGroup const theEdgeInfos1{};
			EdgeGroup const theEdgeInfos2{};

		}; // EdgeGroupDuo

		//! Pairs of EdgeInfo groups
		inline
		std::vector<EdgeGroupDuo>
		foo
			() const
		{
			return {};//TODO
		}
		*/


	}; // EdgeEval


} // [sig]

} // [quadloco]

