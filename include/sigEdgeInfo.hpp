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

#include <cmath>


namespace quadloco
{

namespace sig
{

	//! \brief Attributes of single Edgel
	class EdgeInfo
	{
		//! The edgel for which realtionships are being tracked
		img::Edgel const theEdgel;

		// Tracking information (from consider() function)

		//! Sum of weights suggesting part of a radial edge
		double theWgtRadialSum;

// TODO - handled by collinearity assessment?
//		//! Sum of location differences to (qualified) other edgels
//		img::Vector<double> theDiffDirSum;

		//! Sum of mean alignment directions with other (qualified) edgels
		img::Vector<double> theEdgeDirSum;

	public:

		//! \brief Begin gathering information related to this edge element
		inline
		explicit
		EdgeInfo
			( img::Edgel const & edgel
			)
			: theEdgel{ edgel }
			, theWgtRadialSum{ 0. }
//			, theDiffDirSum{ 0., 0. }
			, theEdgeDirSum{ 0., 0. }
		{ }

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
		img::Vector<double>
		edgeDirection
			() const
		{
			return edgel().direction();
		}

		//! \brief Forward location of Edgel
		inline
		img::Vector<double>
		edgeLocation
			() const
		{
			return edgel().location();
		}

		//! Composite gradient associated edgel pair (positive with ndx1)
		inline
		static
		img::Vector<double>
		alignDirBetween
			( img::Edgel const & edgel1
			, img::Edgel const & edgel2
			)
		{
			img::Vector<double> const & dir1 = edgel1.direction();
			img::Vector<double> const & dir2 = edgel2.direction();
			//
			// NOTE: treat dir1 as positive direction and negate dir2
			//
			img::Vector<double> const sumPosNeg{ dir1 - dir2 };
			img::Vector<double> const meanDir{ direction(sumPosNeg) };
			return meanDir;
		}

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

		/*! \brief Use someEdgel to update tracking information
		 *
		 * Assess how much support someEdgel provides to this instance
		 * in context of this edgel being part of a quad target
		 * - is someEdgel dir almost oppositely directed (opposite radii)
		 * - is someEdgel position nearly collinear (same/opposite radii)
		 */
		// This is probably not a good idea (linked edgels should handle)
		// - is someEdgel dir similarly directed (on same radial line?)
		inline
		void
		consider
			( img::Edgel const & someEdgel
				//!< Adapt this edgel's tracking info based on someEdgel
			, double const & lineGapSigma = 2.
				//!< Std deviation expected in collinearity of opposite radii
			, double const & antiAlignPower = 30.
				//!< Magic number: 30. approx +/-15 deg, 1-sigma (ref code)
			)
		{
			// opposed direction

			// Negative dot product to determine the degree to which
			// the this and someEdgel are facing opposite directions
			double const dotFacing
				{ -dot(theEdgel.direction(), someEdgel.direction()) };

			// skip evaluation of not well anti-aligned edgels
			// (also: code below only valid for positive dotFacing values)
			constexpr double dotFacingMin{ .50 }; // about +/-60 deg window
			if (dotFacingMin < dotFacing)
			{
				// Relative gap between lines defined by both edgels
				double const lineGapDist
					{ averageLineGapDist(theEdgel, someEdgel) };

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

					// Pseudo-probability that this and someEdgel are near same
					// line in space within normal PDF having lineGapSigma
					double const wgtLineGap
						{ weightCollinear(lineGapDist, lineGapSigma) };

					// Pseudo probability that this and someEdge are on opposite
					// radial edges (e.g. of a perspective quad target image)
					double const wgtRadial
						{ wgtFacing * wgtLineGap };

//					// (unitary) direction from this toward other location
//					img::Vector<double> const dirToOther
//						{ direction(someEdgel.start() - theEdgel.start()) };

					// (unitary) "average" direction of this edgel and other
					img::Vector<double> const pairDir
						{ alignDirBetween(theEdgel, someEdgel) };

					// update tracking information
					theWgtRadialSum += wgtRadial;
//					theDiffDirSum = theDiffDirSum + wgtRadial * dirToOther;
					theEdgeDirSum = theEdgeDirSum + wgtRadial * pairDir;
				}
			}
		}

//		//! \brief Angle of gradient direction
//		inline
//		double
//		angleOfDeltaDirSum
//			() const
//		{
//			return ang::atan2(theDiffDirSum[1], theDiffDirSum[0]);
//		}

		//! \brief Angle of gradient direction
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

		//! \brief Current cummulative weight for this edgel
		inline
		double const &
		consideredWeight
			() const
		{
			return theWgtRadialSum;
		}

	}; // EdgeInfo



} // [sig]

} // [quadloco]

