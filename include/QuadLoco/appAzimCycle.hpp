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
#include "imgQuadTarget.hpp"
#include "imgSpot.hpp"
#include "prbStats.hpp"
#include "rasgrid.hpp"
#include "rasGrid.hpp"
#include "valSpan.hpp"

#include <numbers>
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
	 *
	 * The template type is to match the ras::Grid<GridType> source
	 * data with which an instance is constructed.
	 */
	template <typename GridType = float>
	class AzimCycle
	{
		//! Statistics for all source values within evaluation circle
		prb::Stats<double> theSrcStat{};

		//! Location in source image being evaluated as possible quad center
		img::Spot const theEvalCenter{};

		//! Index/angle map (with wrapparound but wrap is not needed here)
		ang::Ring const theAzimRing{};

		//! Statistics for source values along each azimuth direction
		std::vector<prb::Stats<double> > theAzimStats{};

		//! Simple structure for angle and sign of intensity deviation
		struct AzimInten  // AzimCycle::
		{
			//! Tracking deviation
			enum Inten
				{ Unknown // not well defined
				, Hi  // deviation is above expected mean value
				, Lo  // deviation is below expected mean value
				};

			//! Angle of azimuth statistics
			double theAngle{ std::numeric_limits<double>::quiet_NaN() };

			//! Mean value of azimuth stats
			double theMeanValue{ std::numeric_limits<double>::quiet_NaN() };

			//! Classification of azimuth as Hi or Lo
			Inten theHiLo{ Unknown };


			//! String representation of enum Inten
			inline
			static
			std::string
			nameFor  // AzimCycle::AzimInten::
				( Inten const & inten
				)
			{
				std::string str("Unknown");
				if (Hi == inten) { str = "Hi"; }
				else
				if (Lo == inten) { str = "Lo"; }
				return str;
			}

			//! Descriptive information about this instance.
			inline
			std::string
			infoString  // AzimCycle::AzimInten::
				( std::string const & title = {}
				) const
			{
				std::ostringstream oss;
				if (! title.empty())
				{
					oss << title << ' ';
				}
				using engabra::g3::io::fixed;
				oss
					<< ' ' << fixed(theAngle)
					<< ' ' << fixed(theMeanValue)
					<< ' ' << nameFor(theHiLo)
					;

				return oss.str();
			}

		}; // AzimInten

		//! \brief Transition of angle Inten (from Hi to Lo or v.v.)
		struct AzimSwing  // AzimCycle::
		{
			double theAngle
				{ std::numeric_limits<double>::quiet_NaN() };

			double theChangeInMean
				{ std::numeric_limits<double>::quiet_NaN() };

			/*! \brief A null or valid instance (valid if actual transition)
			 *
			 * A transition (swing) is indicated when aiBeg and aiEnd
			 * have different states as (Hi and Lo) or (Lo and Hi). In
			 * this case, intensity deviations (relative to fullMean arg
			 * value) are computed for each of the beg/end values. These
			 * computed deviations are used to (linearly) interpolate a
			 * "fit" angle values at which the deviation can be expected
			 * to be zero.
			 */
			inline
			static
			AzimSwing  // AzimCycle::AzimSwing
			from
				( AzimInten const & aiBeg
				, AzimInten const & aiEnd
				, double const & fullMean
				)
			{
				AzimSwing azimSwing{};

				typename AzimInten::Inten const & hlBeg = aiBeg.theHiLo;
				typename AzimInten::Inten const & hlEnd = aiEnd.theHiLo;

				// Swing requires transition between hi/lo (or lo/hi) states
				if (! (hlEnd == hlBeg))
				{
					// angular locations of transtion beg/end
					double const & angBeg = aiBeg.theAngle;
					double angEnd{ aiEnd.theAngle };
					if (angEnd < angBeg)
					{
						angEnd += ang::piTwo();
					}
					double const aSpan{ angEnd - angBeg }; // always pos

					// intensity deviations at transition beg/end
					double const devBeg{ aiBeg.theMeanValue - fullMean };
					double const devEnd{ aiEnd.theMeanValue - fullMean };
					double const dSpan{ devEnd - devBeg }; // can be +/-
					double const dZero{ 0. - devBeg };

					// estimate interpolated location at angle
					// where (linearly) interpolated deviation is zero
					double const aZero{ (aSpan / dSpan) * dZero };
					double const angleFit
						{ ang::principalAngle(angBeg + aZero) };

					double const & changeInMean = dSpan;
					azimSwing = AzimSwing{ angleFit, changeInMean };
				}
				return azimSwing;
			}

			//! True if all members contain valid data values
			inline
			bool
			isValid  // AzimCycle::AzimSwing
				() const
			{
				return
					(  engabra::g3::isValid(theAngle)
					&& engabra::g3::isValid(theChangeInMean)
					&& (0. != theChangeInMean)
					);
			}

			//! Direction of transition
			inline
			bool
			isFromLoIntoHi  // AzimCycle::AzimSwing
				() const
			{
				// true if theChangeInMean is positive
				return (0. < theChangeInMean);
			}

			//! Direction of transition
			inline
			bool
			isFromHiIntoLo  // AzimCycle::AzimSwing
				() const
			{
				// true if theChangeInMean is negative
				return (theChangeInMean < 0);
			}

			//! Direction vector associated with theAngle
			inline
			img::Vector<double>
			direction  // AzimCycle::AzimSwing
				() const
			{
				double const & angle = theAngle;
				return { std::cos(angle), std::sin(angle) };
			}

			/*! \brief True if theAngle of each is near +/-pi different
			 *
			 * Test is performed by creating two img::Spot on unit circle,
			 * spotA and spotB.  If they are in approximately opposite
			 * directions, more specifically
			 * \arg if(dot(spotA, spotB) < -sqrt(.5))
			 * 
			 * Then the angle between them is tested via:
			 * \arg std::abs(outer(spotA, spotB)) < std::sin(tolAng)
			 *
			 * This function returns the conjunction (&&) of above
			 * conditions.
			 */
			inline
			static
			bool
			nearlyOpposite  // AzimCycle::AzimSwing
				( AzimSwing const & itemA
				, AzimSwing const & itemB
				, double const & tolAng = std::numeric_limits<double>::epsilon()
				)
			{
				bool same{ false };
				double const & angA = itemA.theAngle;
				double const & angB = itemB.theAngle;
				img::Spot const spotA{ std::cos(angA), std::sin(angA) };
				img::Spot const spotB{ std::cos(angB), std::sin(angB) };

				// check if approximately in opposite directions
				constexpr double minDot{ 1. / std::numbers::sqrt2_v<double> };
				double const dotAB{ dot(spotA, spotB) };
				if (dotAB < minDot)
				{
					// check if angle between is less than tolerance angle
					double const outAB{ outer(spotA, spotB) };
					double const tol{ std::sin(tolAng) };
					same = (std::abs(outAB) < std::abs(tol));
				}

				return same;
			}

			//! Descriptive information about this instance.
			inline
			std::string
			infoString  // AzimCycle::AzimSwing::
				( std::string const & title = {}
				) const
			{
				std::ostringstream oss;
				if (! title.empty())
				{
					oss << title << ' ';
				}
				using engabra::g3::io::fixed;
				oss
					<< "angle: " << fixed(theAngle)
					<< ' '
					<< "change: " << fixed(theChangeInMean)
					;
				return oss.str();
			}

		}; // AzimSwing

		//! Collection of significant azimuth classification instances
		struct AzimProfile  // AzimCycle::
		{
			//! Threshold against which to compare individual azim statistics
			double const theFullMean
				{ std::numeric_limits<double>::quiet_NaN() };

			//! Values only at azimuths with significant intensity deviations
			std::vector<AzimInten> theAzimIntens{};

			//! Initialize to consider() data against overall mean.
			inline
			explicit
			AzimProfile  // AzimCycle::AzimProfile::
				( double const & overallMean
					//!< Signifance threshold (e.g. average over all azimuths)
				, std::size_t const & numAzim = 1024u
					//!< Number of azimuths to expect (for optimization)
				)
				: theFullMean{ overallMean }
				, theAzimIntens{}
			{
				theAzimIntens.reserve(numAzim);
			}

			//! \brief Determine if azimStats represent significant Inten
			inline
			typename AzimInten::Inten
			intenFor  // AzimCycle::AzimProfile::
				( prb::Stats<double> const & azimStat
				) const
			{
				typename AzimInten::Inten inten{ AzimInten::Unknown };
				double const azimMin{ azimStat.min() };
				double const azimMax{ azimStat.max() };
				if (theFullMean < azimMin) // +: Higher than theFullMean
				{
					inten = AzimInten::Hi;
				}
				if (azimMax < theFullMean) // -: Lower than theFullMean
				{
					inten = AzimInten::Lo;
				}
				return inten;
			}

			//! Incorporate azimInten if it is significant (not unknown) 
			inline
			void
			consider  // AzimCycle::AzimProfile::
				( AzimInten const & azimInten
				)
			{
				if (! (AzimInten::Unknown == azimInten.theHiLo))
				{
					theAzimIntens.emplace_back(azimInten);
				}
			}

			//! Incorporate azimInten if it is significant (not unknown) 
			inline
			void
			consider  // AzimCycle::AzimProfile::
				( double const & azimAngle
				, prb::Stats<double> const & azimStat
				)
			{
				typename AzimInten::Inten const inten{ intenFor(azimStat) };
				double const meanValue{ azimStat.mean() };
				AzimInten const azimInten{ azimAngle, meanValue, inten };
				consider(azimInten);
			}

			/*! \brief Extract AzimSwing transitions from theAzimIntens.
			 *
			 * This profile AzimInten sequence is condensed by finding the
			 * transition points for which Inten transitions from Hi to Lo
			 * or from Lo to Hi. Note that one of these transitions may be
			 * at then profile end/beg in association with angle wraparound.
			 *
			 * At each transition point, use the associated "From" and "Into"
			 * AzimInten instances to interpolate a "fitAngle" value based
			 * on the "from" angle/radValue and the "into" angle/radValue.
			 */
			inline
			std::vector<AzimSwing>
			allAzimSwings  // AzimCycle::AzimProfile::
				() const
			{
				std::vector<AzimSwing> azimSwings{};

				// wrap around index management
				std::size_t const numSamps{ theAzimIntens.size() };
				ang::Ring const sampRing(numSamps);

				// allocate space: overkill - expect 4
				azimSwings.reserve(2u * numSamps + 1u);

				// loop over profile (with wrap)
				for (std::size_t ndxBeg{0u} ; ndxBeg < numSamps ; ++ndxBeg)
				{
					std::size_t const ndxEnd
						{ sampRing.indexRelativeTo(ndxBeg, +1) };

					AzimInten const & aiBeg = theAzimIntens[ndxBeg];
					AzimInten const & aiEnd = theAzimIntens[ndxEnd];

					// check for transition
					AzimSwing const azimSwing
						{ AzimSwing::from(aiBeg, aiEnd, theFullMean) };
					if (azimSwing.isValid())
					{
						azimSwings.emplace_back(azimSwing);
					}
				}

				return azimSwings;
			}

		}; // AzimProfile

		/*! \brief Define an azimuth index/value map based on radius size
		 *
		 * The returned buffer map contains a number of bins that provide
		 * approximately azimuth resolution of approximately 1 source cell
		 * at a distance, radius, from the (ctor) center evaluation point.
		 */
		inline
		static
		ang::Ring
		azimRing  // AzimCycle::
			( double const & radius
				//!< With angle increment of 1 element at this radius
			)
		{
			// number of samples for unit sampling of perimeter at radius
			double const perim{ ang::piTwo()*radius };
			std::size_t const numSamp{ (std::size_t)std::floor(perim) };
			return ang::Ring(numSamp);
		}

	public:

		//! Construct statistics needed inside hasQuadTransition() member.
		inline
		AzimCycle  // AzimCycle::
			( ras::Grid<GridType> const & srcGrid
				//!< Source values with which to evaluate
			, img::Spot const & evalCenter
				//!< Spot about which to evaluate azimuth intensity cycles
			, double const & evalRadius = 7.0
				//!< max radius of evaluation space
			, double const & evalMinRad = 2.5
				//!< min radius (skip if less than this)
			)
			: theSrcStat{}
			, theEvalCenter{ evalCenter }
			, theAzimRing(azimRing(evalRadius))
			, theAzimStats(theAzimRing.size(), prb::Stats<double>{})
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
						img::Spot const sampSpot{ relSpot + theEvalCenter };
						using ras::grid::bilinValueAt;
						double const sampValue
							{ (double)bilinValueAt<GridType>
								(srcGrid, sampSpot)
							};

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
		 * either Hi or Lo or Unknown. Azimuths associated with
		 * Unknown are ignored. Considering the remaining azimuth
		 * classifications in angle order then produces a sequence of
		 * Hi and Lo values.
		 */
		inline
		AzimProfile
		fullAzimProfile  // AzimCycle::
			() const
		{
			// evaluate "run-lengths" for which min/max azimuth statistics
			// are above/below (respectively) the overall patch mean value.
			double const fullMean{ theSrcStat.mean() };
			std::size_t const numAzim{ theAzimRing.size() };
			AzimProfile azimProfile(fullMean, numAzim);

			// check each azimuth - add significant deviations to profile
			for (std::size_t aNdx{0u} ; aNdx < numAzim ; ++aNdx)
			{
				double const angle{ theAzimRing.angleAtIndex(aNdx) };
				prb::Stats<double> const & azimStat = theAzimStats[aNdx];
				azimProfile.consider(angle, azimStat);

			} // aNdx loop

			return azimProfile;
		}

		/*! \brief True if profile sequence is consistent with quad pattern
		 *
		 * Looks for exactly four transitions in the AzimProfile array.
		 *
		 * If this is true, then the four individual transitions are
		 * evaluated to determine if the (interpolated) angle crossings
		 * have approximate half-turn symmetry between each pair.
		 *
		 * I.e. there should be four transitions, (a,b,c,d), in any true
		 * quad pattern it is expected that (a & c) are about 1/2 turn
		 * apart from each other and that (b & d) are also about one half
		 * turn different from each other.
		 */
		inline
		bool
		hasQuadTransitions  // AzimCycle::
			( AzimProfile const & azimProfile
				//!< Such as returned from azimProfile()
			, double const & tolNumAzimDelta = 2.
				//!< Number theAzimRing.angleDelta() for symmetry tolerance
			) const
		{
			bool quadlike{ false };

			// get profile transitions (between 'Hi' and 'Lo' regions)
			std::vector<AzimSwing> const swings{ azimProfile.allAzimSwings() };

			// since quad-like azimuth data should have alternating pattern
			// of intensity transitions, a legitimate quad target should
			// therefore have four distinct transitions.
			if (4u == swings.size())
			{
				// if there are a correct number of transitions, then
				// check if the transition angular positions occur
				// in pairs that are nearly a half turn different.
				double const tol{ tolNumAzimDelta * theAzimRing.angleDelta() };
				if ( AzimSwing::nearlyOpposite(swings[0], swings[2], tol)
				  && AzimSwing::nearlyOpposite(swings[1], swings[3], tol)
				   )
				{
					quadlike = true;
				}
			}

			return quadlike;
		}

		/*! \brief True if ctor's srcGrid has quad-like azimuth transitions.
		 *
		 * A valid quad image pattern, when considered from the quad
		 * center, is expected to have a characteristic azimuthal intensity
		 * variation. I.e. relative to the local srcGrid area mean
		 * intensity, azimuth sampling of source values should produce
		 * one of several characteristic "Hi,Lo,Hi,Lo (or Lo,Hi,Lo,Hi)
		 * style variations. (ref fullAzimProfile()).
		 *
		 * This is the primary member function for general use. I.e.
		 * \arg First construct an instance (which computes needed
		 *      statistics)
		 * \arg Then call this funtion to determine if four transitions
		 *      are likely present in the source image (as viewed from
		 *      the evalCenter location used in ctor.
		 *
		 * This function is equivalent to calling the composite
		 * \arg hasQuadTransitions(fullAzimProfile())
		 */
		inline
		bool
		hasQuadTransitions  // AzimCycle::
			() const
		{
			return hasQuadTransitions(fullAzimProfile());
		}

		//! Quad target image parameters estimated from azimuth profile
		inline
		img::QuadTarget
		imgQuadTarget  // AzimCycle::
			() const
		{
			img::QuadTarget imgQuad{};

			// get profile transitions (between 'Hi' and 'Lo' regions)
			AzimProfile const azimProfile{ fullAzimProfile() };
			std::vector<AzimSwing> const swings{ azimProfile.allAzimSwings() };

			// since quad-like azimuth data should have alternating pattern
			// of intensity transitions, a legitimate quad target should
			// therefore have four distinct transitions.
			if (4u == swings.size())
			{
				// get the four radial edge transitions
				AzimSwing const & itemAp = swings[0];
				AzimSwing const & itemBp = swings[1];
				AzimSwing const & itemAn = swings[2];
				AzimSwing const & itemBn = swings[3];

				if ( theEvalCenter.isValid()
				  && itemAp.isValid()
				  && itemAn.isValid()
				  && itemBp.isValid()
				  && itemBn.isValid()
				   )
				{
					// define four spots at transitions on unit circle
					// (these are expected to be in positive angle order!)
					img::Vector<double> const spotAp{ itemAp.direction() };
					img::Vector<double> const spotBp{ itemBp.direction() };
					img::Vector<double> const spotAn{ itemAn.direction() };
					img::Vector<double> const spotBn{ itemBn.direction() };

					// compute average radial directions from antipodal pairs
					// of the unit circle spots
					img::Vector<double> const sumA{ spotAp - spotAn };
					img::Vector<double> const sumB{ spotBp - spotBn };
					img::Vector<double> const dirA{ direction(sumA) };
					img::Vector<double> const dirB{ direction(sumB) };

					// dirX is (Hi to right, and Lo to left) (azimSwing Hi->Lo)
					// dirY is adjacent to dirX in positive rotation sense
					if (itemAp.isFromHiIntoLo())
					{
						img::Vector<double> const & dirX = dirA;
						img::Vector<double> const & dirY = dirB;
						imgQuad = img::QuadTarget{ theEvalCenter, dirX, dirY };
					}
					else
					if (itemAp.isFromLoIntoHi())
					{
						img::Vector<double> const & dirX = dirB;
						img::Vector<double> const dirY{ -dirA };
						imgQuad = img::QuadTarget{ theEvalCenter, dirX, dirY };
					}
					// else no change across itemAp
				}
			}

			return imgQuad;
		}


	}; // AzimCycle



} // [app]

} // [quadloco]

