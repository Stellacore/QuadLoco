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
 * \brief Declarations for quadloco::sig::EdgeInfo namespace
 *
 */


#include "imgEdgel.hpp"

#include <Engabra>

#include <cmath>


namespace quadloco
{

namespace sig
{

	//! \brief Attributes of single Edgel
	class EdgeInfo
	{
		//! The edgel for which realtionships are being tracked
		img::Edgel const theEdgel{};

		// Tracking information (from considerOther() function)

		//! Sum of weights suggesting part of a radial edge
		double theWgtRadialSum{ engabra::g3::null<double>() };

		//! Sum of mean alignment directions with other (qualified) edgels
		img::Vector<double> theEdgeDirSum{};

	// static methods

		//! \brief Average distance of other edge location from own edge line.
		inline
		static
		double
		averageLineGapDist
			( img::Edgel const & edgel1
			, img::Edgel const & edgel2
			)
		{
			// separation of the two lines
			img::Vector<double> const & loc1 = edgel1.start();
			img::Vector<double> const & loc2 = edgel2.start();
			img::Vector<double> const & dir1 = edgel1.direction();
			img::Vector<double> const & dir2 = edgel2.direction();
			// distance of loc2 from line through edgel1
			double const dist2from1{ dot((loc2 - loc1), dir1) };
			// distance of loc1 from line through edgel2
			double const dist1from2{ dot((loc1 - loc2), dir2) };
			// average of distances from each other
			double const distBetween{ .5 * (dist2from1 + dist1from2) };
			return distBetween;
		}

		//! \brief Pseudo-probability of being on the same line given sigma
		inline
		static
		double
		weightCollinear
			( double const & lineGapDist
				//!< Separation of lines between two edgels
			, double const & lineGapSigma
				//!< Uncertainty in collinearity (tolerance is 3x this)
			)
		{
			double wgt{ 0. };
			double const lineGapMax{ 4. * lineGapSigma };
			if (lineGapDist < lineGapMax)
			{
				// assign pseudo weight on collinearity
				double const arg{ lineGapDist / lineGapSigma };
				double const wgtLineGap{ std::exp(-arg*arg) };
				wgt = wgtLineGap;
			}
			return wgt;
		}

	// private

		//! Composite gradient associated edgel pair (positive with ndx1)
		inline
		img::Vector<double>
		directionAdjustedTo
			( img::Edgel const & other
			) const
		{
			img::Vector<double> const & dirSelf = theEdgel.direction();
			img::Vector<double> const & dirOther = other.direction();
			//
			// NOTE: treat dirSelf as positive direction and negate dirOther
			//
			img::Vector<double> const negOther{ -dirOther };
			img::Vector<double> const sumDirs{ dirSelf + negOther };
			img::Vector<double> const meanDir{ direction(sumDirs) };
			return meanDir;
		}

		//! Composite gradient associated edgel pair (positive with ndx1)
		inline
		img::Vector<double>
		directionForLocation
			( img::Edgel const & other
			) const
		{
			// reference direction (for plus/minus)
			img::Vector<double> const & dirSelf = theEdgel.direction();

			img::Vector<double> const & locSelf = theEdgel.start();
			img::Vector<double> const & locOther = other.start();
			img::Vector<double> const diffDir{ direction(locOther - locSelf) };
			img::Vector<double> const perpDir{ -diffDir[1], diffDir[0] };
			double sgn{ 1. };
			if (dot(dirSelf, perpDir) < 0.)
			{
				sgn = -1.;
			}
			img::Vector<double> const edgeDir{ sgn * perpDir };
			return edgeDir;
		}

	public:

		//! Contruct a null (! isValid()) instance
		inline
		explicit
		EdgeInfo
			() = default;

		//! \brief Begin gathering information related to this edge element
		inline
		explicit
		EdgeInfo
			( img::Edgel const & edgel
			)
			: theEdgel{ edgel }
			, theWgtRadialSum{ 0. }
			, theEdgeDirSum{ 0., 0. }
		{ }

		//! True if this instance contains valid (not null) data
		inline
		bool
		isValid
			() const
		{
			return
				(  theEdgel.isValid()
				&& engabra::g3::isValid(theWgtRadialSum)
				&& theEdgeDirSum.isValid()
				);
		}

		//
		// Direct access to edgel components
		//

		//! \brief Edge element
		inline
		img::Edgel const &
		edgel
			() const
		{
			return theEdgel;
		}

		//! \brief Forward direction from Edgel
		inline
		img::Vector<double> const &
		edgeDirection
			() const
		{
			// img::Edgel inherits from img::Ray with (const &) direction()
			return theEdgel.direction();
		}

		//! \brief Forward location of Edgel
		inline
		img::Vector<double> const &
		edgeLocation
			() const
		{
			// img::Edgel inherits from img::Ray with (const &) start()
			return theEdgel.start();
		}

		//
		// Accumulation/tracking functions
		//

		/*! \brief Use otherEdgel to update tracking information
		 *
		 * Assess how much support otherEdgel provides to this instance
		 * in context of this edgel being part of a quad target
		 * - is otherEdgel dir almost oppositely directed (opposite radii)
		 * - is otherEdgel position nearly collinear (same/opposite radii)
		 */
		// This is probably not a good idea (linked edgels should handle)
		// - is otherEdgel dir similarly directed (on same radial line?)
		inline
		void
		considerOther
			( img::Edgel const & otherEdgel
				//!< Adapt this edgel's tracking info based on otherEdgel
			, double const & lineGapSigma = 2.
				//!< Std deviation expected in collinearity of opposite radii
			, double const & antiAlignPower = 30.
				//!< Magic number: 30. approx +/-15 deg, 1-sigma (ref code)
			)
		{
			// opposed direction

			// Negative dot product to determine the degree to which
			// the this and otherEdgel are facing opposite directions
			double const dotFacing
				{ -dot(theEdgel.direction(), otherEdgel.direction()) };

			// skip evaluation of not well anti-aligned edgels
			// (also: code below only valid for positive dotFacing values)
			constexpr double dotFacingMin{ .50 }; // about +/-60 deg window
			if (dotFacingMin < dotFacing)
			{
				// Relative gap between lines defined by both edgels
				double const lineGapDist
					{ averageLineGapDist(theEdgel, otherEdgel) };

				// don't waste time computing on large gaps
				double const lineGapMax{ 4. * lineGapSigma };
				if (lineGapDist < lineGapMax)
				{
					// (arbitrarily) use cos()^N as weighting function for which
					// N=30 approximates exp(-(x/.25)^2) which is a Gaussian
					// with (0.25 == sigma) or about +/- 15deg for 1-sigma, and
					// +/- 45 deg for 3-sigma excursions
					double const wgtFacing
						{ std::pow(dotFacing, antiAlignPower) };

					// Pseudo-probability that this and otherEdgel are near same
					// line in space within normal PDF having lineGapSigma
					double const wgtLineGap
						{ weightCollinear(lineGapDist, lineGapSigma) };

					// Pseudo prob that this and otherEdge are on opposite
					// radial edges (e.g. of a perspective quad target image)
					double const wgtRadial
						{ wgtFacing * wgtLineGap };

					// (unitary) "average" direction of this edgel and other
					img::Vector<double> const pairDir
						{ directionAdjustedTo(otherEdgel) };
					// this tends to bias edges for wide responses
					// e.g. two cell wide edges create a circular pattern
					//	{ directionForLocation(otherEdgel) };

					// update tracking information
					theWgtRadialSum += wgtRadial;
					theEdgeDirSum = theEdgeDirSum + wgtRadial * pairDir;

/*
using engabra::g3::io::fixed;
std::cout
	<< "wgtFacing: " << fixed(wgtFacing)
	<< ' '
	<< "wgtLineGap: " << fixed(wgtLineGap)
	<< ' '
	<< "wgtRadial: " << fixed(wgtRadial)
	<< '\n';
*/

				}
			}
		}

		//! \brief Angle associated with accumulted (pairwise) edge directions
		inline
		double
		consideredAngle
			() const
		{
			return ang::atan2(theEdgeDirSum[1], theEdgeDirSum[0]);
		}

		//! \brief Direction (unitary) of pairwise mean edge directions
		inline
		img::Vector<double>
		consideredDirection
			() const
		{
			return direction(theEdgeDirSum);
		}

		/*! \brief Current cummulative significance weight for this edgel
		 *
		 * As candidate for belonging to a (quad target) radial edge.
		 */
		inline
		double const &
		consideredWeight
			() const
		{
			return theWgtRadialSum;
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
			using engabra::g3::io::fixed;
			oss
				<< "edgel: " << theEdgel
				<< ' '
				<< "wgtRad: " << fixed(theWgtRadialSum)
				<< ' '
				<< "dirSum: " << theEdgeDirSum
				;
			return oss.str();
		}


	}; // EdgeInfo


} // [sig]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::sig::EdgeInfo const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::sig::EdgeInfo const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

