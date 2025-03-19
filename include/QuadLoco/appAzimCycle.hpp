//
// MIT License
//
// Copyright (c) 2025 Stellacore Corporation
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
 * \brief Declarations for quadloco::app::AzimCycle namespace
 *
 */


#include "ang.hpp"
#include "angRing.hpp"
#include "imgSpot.hpp"
#include "prbStats.hpp"
#include "rasgrid.hpp"
#include "rasGrid.hpp"

#include <vector>


namespace quadloco
{

namespace app
{

	/*! \brief Evaluation class to test for 'quad-like' pattern at a location.
	 *
	 * Quad target images are associated with a distinctive intensity
	 * pattern that varies with azimuth relative to the center point.
	 * This class provides the method, hasQuadTransitions(), that checks
	 * if intensity values from a source image follow the expected pattern.
	 */
	class AzimCycle
	{
		//! Statistics for all source values within evaluation circle
		quadloco::prb::Stats<double> theSrcStat{};

		//! Index/angle map (with wrapparound but wrap is not needed here)
		quadloco::ang::Ring const theAzimRing{};

		//! Statistics for source values along each azimuth direction
		std::vector<quadloco::prb::Stats<double> > theAzimStats{};

		/*! \brief Define an azimuth index/value map based on radius size
		 *
		 * The returned buffer map contains a number of bins that provide
		 * approximately azimuth resolution of approximately 1 source cell
		 * at a distance, radius, from the (ctor) center evaluation point.
		 */
		inline
		static
		quadloco::ang::Ring
		azimRing
			( double const & radius
				//!< With angle increment of 1 element at this radius
			)
		{
			// number of samples for unit sampling of perimeter at radius
			double const perim{ quadloco::ang::piTwo()*radius };
			std::size_t const numSamp{ (std::size_t)std::floor(perim) };
			return quadloco::ang::Ring(numSamp);
		}

	public:

		//! Construct statistics needed inside hasQuadTransition() member.
		inline
		AzimCycle
			( quadloco::ras::Grid<float> const & srcGrid
				//!< Source values with which to evaluate
			, quadloco::img::Spot const & evalCenter
				//!< Spot about which to evaluate azimuth intensity cycles
			, double const & evalRadius = 7.0
				//!< max radius of evaluation space
			, double const & evalMinRad = 2.5
				//!< min radius (skip if less than this)
			)
			: theSrcStat{}
			, theAzimRing(azimRing(evalRadius))
			, theAzimStats(theAzimRing.size(), quadloco::prb::Stats<double>{})
		{
			// sample a circular patch from the source image. Compute the
			// angle for each source pixel and accumulate its value into
			// the appropriate azimuth statistics bin.
			double const rcMax{ evalRadius + .5 };
			for (double dr{-evalRadius} ; dr < rcMax ; dr += 1.)
			{
				for (double dc{-evalRadius} ; dc < rcMax ; dc += 1.)
				{
					using namespace quadloco;

					// relative sample location w.r.t. evaluation center
					img::Spot const relSpot{ dr, dc };
					double const sampRadius{ magnitude(relSpot) };
					if (! (sampRadius < evalMinRad)) // inside eval circle
					{
						// determine angle of sample w.r.t. evaluation center
						double const sampAngle
							{ ang::atan2(relSpot[1], relSpot[0]) };

						// extract (interpolated) source image value
						img::Spot const sampSpot{ relSpot + evalCenter };
						using ras::grid::bilinValueAt;
						double const sampValue
							{ (double)bilinValueAt<float>(srcGrid, sampSpot) };

						// add sample into azimuthal statistics collection
						std::size_t const azimNdx
							{ theAzimRing.indexFor(sampAngle) };
						theAzimStats[azimNdx].consider(sampValue);
						theSrcStat.consider(sampValue);
					}
				}
			}
		}

		/*! \brief Statistically strong transitions around full circle.
		 *
		 * The azimuth statistics (theAzimStats) are evaluated to
		 * determine significant transitions relative to the overall
		 * source statistics (theSrcStat).  Transitions are defined as
		 * a change from a Hi value to a Lo value or vice versa,
		 * with Hi/Lo defined as:
		 * \arg Hi: all values along an azimuth are larger than
		 *      the overall patch mean value.
		 * \arg Lo: all values along an azimuth are smaller than
		 *      the overall patch mean value.
		 *
		 * Each azimuth in theAzimStats is evaluated and classified as
		 * either Hi or Lo or (not Hi/Lo). Azimuths associated with "not
		 * Hi/Lo" are ignored. Considering the remaining azimuth
		 * classifications in angle order then produces a sequence of
		 * Hi and Lo values. This sequence is condensed by removing
		 * duplicate adjacent Hi and duplicate adjacent Lo values. The
		 * remaining values characterizes a sequence of transitions that
		 * are statistically significant. (e.g. Hi,Lo,Hi,Lo,Hi).
		 *
		 * The return vector contains a representation of this Hi/Lo
		 * sequence that is encoded as +/-1 values (+1:Hi, -1:Lo).
		 */
		inline
		std::vector<int>
		azimHiLoSigns
			() const
		{
			std::vector<int> signs{};
			// evaluate "run-lengths" for which min/max azimuth statistics
			// are above/below (respectively) the overall patch mean value.
			double const fullMean{ theSrcStat.mean() };
			std::size_t const numAzim{ theAzimRing.size() };
			signs.reserve(numAzim);
			int prevSign{ 0 };
			for (std::size_t aNdx{0u} ; aNdx < numAzim ; ++aNdx)
			{
				using namespace quadloco;

				prb::Stats<double> const & azimStat = theAzimStats[aNdx];
				double const azimMin{ azimStat.min() };
				double const azimMax{ azimStat.max() };

				int currSign{ 0 };
				if (fullMean < azimMin) // +
				{
					currSign = +1;
				}
				if (azimMax < fullMean) // -
				{
					currSign = -1;
				}

				// only significant signs are of interest
				if (! (0 == currSign))
				{
					// update signs array
					if ( signs.empty()   // start with first significant sign
					  || (prevSign != currSign) // and add any change in sith
					   )
					{
						signs.emplace_back(currSign);
						prevSign = currSign;

					}
				}
			} // aNdx loop

			return signs;
		}

		/*! \brief True if the sequence as four transitios. Ref azimHiLoSigns()
		 *
		 * Looks for four transitions in the azimSigns array. I.e. 
		 * Assuming the azimSigns array is condensed (as is produced
		 * by azimHiLoSigns() method) then this array contains a sequence
		 * of alternating signs and the size of the array indicates the
		 * number of transitions. E.g.
		 * \arg 5u==size: This is the general case, in which the azimuth
		 *      sampling begins in the middle of a Hi(or Lo) region then
		 *      transitions 4x times to end again in the Hi(or Lo) region.
		 *      E.g. (Hi,Lo,Hi,Lo,Hi) or (Lo,Hi,Lo,Hi,Lo)
		 * \arg 4u==size: This is the special case situation in which the
		 *      azimuth sampling begins on/near a quad image radial edge
		 *      and only full Hi/Lo intervals are produced.
		 *      E.g. (Hi,Lo,Hi,Lo) or (Lo,Hi,Lo,Hi)
		 */
		inline
		bool
		hasQuadTransitions
			( std::vector<int> const & azimSigns
				//!< Such as returned from azimHiLoSigns()
			) const
		{
			// Quad-like azimuth data should have alternating pattern of
			// signs (e.g. (-1,+1,-1,+1,-1) or (+1,-1,+1,-1)
			// less than 4 or greater than 5 transitions (between high/low
			// values) violates the expected quad pattern symmetry.
			return
				{  (5u == azimSigns.size())  // general case
				|| (4u == azimSigns.size())  // special case
				};
		}

		/*! \brief True if ctor's srcGrid has quad-like azimuth transitions.
		 *
		 * A valid quad image pattern, when considered from the quad
		 * center, is expected to have a characteristic azimuthal intensity
		 * variation. I.e. relative to the local srcGrid area mean
		 * intensity, azimuth sampling of source values should produce
		 * one of several characteristic "Hi,Lo,Hi,Lo (or Lo,Hi,Lo,Hi)
		 * style variations. (ref azimHiLoSigns()).
		 *
		 * This is the primary member function for general use. I.e.
		 * \arg First construct an instance (which computes needed
		 *      statistics)
		 * \arg Then call this funtion to determine if four transitions
		 *      are likely present in the source image (as viewed from
		 *      the evalCenter location used in ctor.
		 *
		 * This function is equivalent to calling the composite
		 * \arg hasQuadTransitions(azimHiLoSigns())
		 */
		inline
		bool
		hasQuadTransitions
			() const
		{
			std::vector<int> const signs{ azimHiLoSigns() };
			return hasQuadTransitions(signs);
		}


	}; // AzimCycle



} // [app]

} // [quadloco]

