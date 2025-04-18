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
 * \brief Declarations for quadloco::app namespace functions
 *
 */


#include "QuadLoco/opsAllPeaks2D.hpp"
#include "QuadLoco/opsCenterRefinerSSD.hpp"
#include "QuadLoco/opsSymRing.hpp"
#include "QuadLoco/prbStats.hpp"
#include "QuadLoco/rasgrid.hpp"
#include "QuadLoco/rasGrid.hpp"
#include "QuadLoco/rasPeakRCV.hpp"

#include <vector>


namespace quadloco
{

namespace app
{

namespace center
{
	//! Intensity value centroid in annulus about evalCenter
	template <typename GridType>
	inline
	img::Spot
	valueCentroid
		( ras::Grid<GridType> const & srcGrid
			//!< Source values with which to evaluate
		, img::Spot const & evalCenter
			//!< Spot about which to evaluate azimuth intensity cycles
		, double const & evalMaxRad = 7.0
			//!< max radius of evaluation space
		, double const & evalMinRad = 2.5
			//!< min radius (skip if less than this)
		)
	{
		img::Vector<double> centroid{};

		// weighted centroid of patch should be near eval center
		img::Vector<double> sumLocs{ 0., 0. };
		double sumInten{ 0. };

		// sample a circular patch from the source image. Compute the
		// angle for each source pixel and accumulate its value into
		// the appropriate azimuth statistics bin.
		double const rcMax{ evalMaxRad + .5 };
		for (double dr{-evalMaxRad} ; dr < rcMax ; dr += 1.)
		{
			for (double dc{-evalMaxRad} ; dc < rcMax ; dc += 1.)
			{
				using namespace quadloco;

				// relative sample location w.r.t. evaluation center
				img::Spot const relSpot{ dr, dc };
				double const sampRadius{ magnitude(relSpot) };
				if (! (sampRadius < evalMinRad)) // inside eval circle
				{
					// extract (interpolated) source image value
					img::Spot const sampSpot{ relSpot + evalCenter };
					using ras::grid::bilinValueAt;
					double const sampValue
						{ (double)bilinValueAt<GridType>
							(srcGrid, sampSpot)
						};

					// update centroid tracking sums
					if (engabra::g3::isValid(sampValue))
					{
						sumLocs  = sumLocs + sampValue * sampSpot;
						sumInten = sumInten + sampValue;
					}
				}
			}
		}

		// compute centroid from running sums
		if (0. < sumInten)
		{
			centroid = (1./sumInten) * sumLocs;
		}
		return img::Spot{ centroid };
	}

	/*! \brief Peaks from application of multiple combined symmetry filters
	 *
	 * The halfSize values define the collection of SymRing filters that
	 * are to be run. The first halfSize filter is run across the entire
	 * srcGrid.  Peak finding is performed on the result (generally 
	 * producing many peaks). Larger halfSizes generally produce stronger
	 * and fewer peaks, but are slightly more expensive to evaluate.
	 *
	 * For each of the numFirstPeaks initial peak detections, the peak
	 * location is used as evaluation point for the remaining SymRing
	 * filters. The filter responses are combined (multiplicatively)
	 * to produce the peak value in the returned peakRCV collection.
	 *
	 * The grid values are determined using ops::SymRing response at
	 * each source grid input pixel. Criteria include a combination
	 * of 1) balance between dark and light pixels; 2) high contrast
	 * in the local area; and 3) symmetry of radiometric values under
	 * a half-turn rotation.
	 */
	inline
	std::vector<ras::PeakRCV>
	multiSymRingPeaks
		( ras::Grid<float> const & srcGrid
			//!< Input intensity grid
		, prb::Stats<float> const & srcStats
			//!< Statisics for srcGrid values
		, std::vector<std::size_t> const & ringHalfSizes
			//!< SymRing quantized radius - in order of application.
		)
	{
		std::vector<ras::PeakRCV> peakCombos;

		bool const srcOkay{ srcGrid.isValid() && srcStats.isValid() };

		if (srcOkay && (! ringHalfSizes.empty()))
		{
			// construct filter objects with requested geometry
			// (e.g. define relative row/col offsets from filter origin)
			std::vector<ops::SymRing> symRings;
			symRings.reserve(ringHalfSizes.size());
			for (std::size_t const & ringHalfSize : ringHalfSizes)
			{
				ops::SymRing const symRing(&srcGrid, srcStats, ringHalfSize);
				symRings.emplace_back(symRing);
			}

			// run initial symmetry filter
			ops::SymRing const & symRingA = symRings.front();
			ras::Grid<float> const peakGridA
				{ ops::symRingGridFor(srcGrid, symRingA) };

			// get all peaks from gridA
			ops::AllPeaks2D const allPeaksA(peakGridA);
			std::size_t const numToGet{ srcGrid.size() }; // { 100u };
			std::vector<ras::PeakRCV> const peakAs
				{ allPeaksA.largestPeakRCVs(numToGet) };
			if (! peakAs.empty())
			{
				// qualify 'A' peaks using symmetry response of 'B'
				peakCombos.reserve(peakAs.size());
				for (ras::PeakRCV const & peakA : peakAs)
				{
					std::size_t const & row = peakA.theRowCol.row();
					std::size_t const & col = peakA.theRowCol.col();
					double valueCombo{ peakA.theValue };
					for (std::size_t nn{1u} ; nn < symRings.size() ; ++nn)
					{
						ops::SymRing const & symRingB = symRings[nn];
						float const valueB{ symRingB(row, col) };
						valueCombo *= static_cast<double>(valueB);

/*
using engabra::g3::io::fixed;
std::cout
	<< "values{A,B,combo}:"
	<< ' ' << fixed(peakA.theValue)
	<< ' ' << fixed(valueB)
	<< ' ' << fixed(valueCombo)
	<< '\n';
*/

					}
					float const fVal{ static_cast<float>(valueCombo) };
				//	peakCombos.emplace_back(ras::PeakRCV{ { row, col }, fVal });
					peakCombos.emplace_back
						(ras::PeakRCV{ peakA.theRowCol, fVal });

				} // peakAs

				std::sort(peakCombos.rbegin(), peakCombos.rend());

			} // ! peakAs.empty()

		} // if (srcOkay && (! ringHalfSizes.empty()))

		return peakCombos;
	}

	//! \brief Convenience version if srcStatistics are already avilable
	inline
	std::vector<ras::PeakRCV>
	multiSymRingPeaks
		( ras::Grid<float> const & srcGrid
			//!< Input intensity grid
		, std::vector<std::size_t> const & ringHalfSizes
			//!< SymRing quantized radius - in order of application.
		)
	{
		// get input grid statistics
		prb::Stats<float> const srcStats(srcGrid.cbegin(), srcGrid.cend());

		return multiSymRingPeaks(srcGrid, srcStats, ringHalfSizes);
	}


	//! Refined center hit via multiSymRingPeaks and CenterRefinerSSD.
	inline
	img::Hit
	refinedHitFrom
		( ras::Grid<float> const & srcGrid
		, std::vector<std::size_t> const & halfRingSizes
		)
	{
		img::Hit centerHit;

		// find nominal peaks
		std::vector<ras::PeakRCV> peakRCVs
			{ app::center::multiSymRingPeaks(srcGrid, halfRingSizes) };

		if (! peakRCVs.empty())
		{
			// refine peak with max value
			std::vector<ras::PeakRCV>::const_iterator const itMax
				{ std::max_element(peakRCVs.cbegin(), peakRCVs.cend()) };
			if (peakRCVs.cend() != itMax)
			{
				ras::PeakRCV const & peakRCV = *itMax;
				ops::CenterRefinerSSD const refiner(&srcGrid);
				centerHit = refiner.fitHitNear(peakRCV.theRowCol);
			}
		}
		return centerHit;
	}

} // [center]


} // [app]

} // [quadloco]

