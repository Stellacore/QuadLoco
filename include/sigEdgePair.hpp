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
 * \brief Declarations for quadloco::sig::EdgePair namespace
 *
 */


#include "ang.hpp"
#include "imgGrad.hpp"
#include "imgSpot.hpp"
#include "sigEdgeInfo.hpp"

#include <cmath>
#include <vector>


namespace quadloco
{

/*! \brief Namespaced functions and utilities for TODO
 */
namespace sig
{

	//! \brief Edgel pair attributes for candidates on opposite radial edges
	class EdgePair
	{
		std::size_t theNdx1; //!< First index into external edge array
		std::size_t theNdx2; //!< Second index into external edge array
		double theDotFacing; //!< Negative dot product of edge gradients
		double theWgtFacing; //!< Pseudo prop of oppositely directed edges
		double theLineGapDist; //!< Distance between edge line segments
		double theWgtLineGap; //!< Pseudo prop of edgels being collinear

		//! Negative of dot product between the two gradient edges
		inline
		static
		double
		dotGradDirs
			( std::size_t const & ndx1
			, std::size_t const & ndx2
			, std::vector<EdgeInfo> const & edgeInfos
			)
		{
			EdgeInfo const & ei1 = edgeInfos[ndx1];
			EdgeInfo const & ei2 = edgeInfos[ndx2];
			return dot(ei1.edgeDirection(), ei2.edgeDirection());
		}

		//! Average distance of other edge location from own edge line.
		inline
		static
		double
		lineGapDist
			( std::size_t const & ndx1
			, std::size_t const & ndx2
			, std::vector<EdgeInfo> const & edgeInfos
			)
		{
			// separation of the two lines
			EdgeInfo const & ei1 = edgeInfos[ndx1];
			EdgeInfo const & ei2 = edgeInfos[ndx2];
			img::Spot const spot1{ ei1.edgeLocation() };
			img::Spot const spot2{ ei2.edgeLocation() };
			img::Grad const dir1{ ei1.edgeDirection() };
			img::Grad const dir2{ ei2.edgeDirection() };
			double const dist2from1{ dot((spot2 - spot1), dir1) };
			double const dist1from2{ dot((spot1 - spot2), dir2) };
			double const distBetween{ .5 * (dist2from1 + dist1from2) };
			return distBetween;
		}

		/*! \brief Pseudo-probability that edges are facing each other.
		 *
		 * (arbitrarily) use cos()^N as weighting function for which
		 * N=30 approximates exp(-(x/.25)^2) which is a Gaussian
		 * with (0.25 == sigma) or about +/- 15deg for 1-sigma, and
		 * +/- 45 deg for 3-sigma excursions
		 */
		inline
		static
		double
		weightFacing
			( double const & dotAlign
				//!< Negative of dot product of the two edgel gradients
			)
		{
			return std::pow(dotAlign, 30.);
		}

		//! Pseudo-probability of being on the same line given sigma
		inline
		static
		double
		weightCollinear
			( double const & lineGapDist
				//!< Separation of lines between two edgels
			, double const & lineGapSigma
				//!< Uncertainty in collinearity (tolerance is 3x this)
			, double const & theDotFacing
				//!< Relative (anti)alignment used to qualify computation
			, double const & theDotFacingMin = .75
			)
		{
			double wgt{ 0. };
			if (theDotFacingMin < theDotFacing)
			{
				double const lineGapMax{ 4. * lineGapSigma };
				if (lineGapDist < lineGapMax)
				{
					// assign pseudo weight on collinearity
					double const arg{ lineGapDist / lineGapSigma };
					double const wgtLineGap{ std::exp(-arg*arg) };
					wgt = wgtLineGap;
				}
			}
			return wgt;
		}

	public:

		//! Compute interesting properties/statistics for pair of edgels.
		inline
		explicit
		EdgePair
			( std::size_t const & ndx1
			, std::size_t const & ndx2
			, std::vector<EdgeInfo> const & edgeInfos
			, double const & lineGapSigma = 2.
			)
			: theNdx1{ ndx1 }
			, theNdx2{ ndx2 }
			, theDotFacing{ -dotGradDirs(theNdx1, theNdx2, edgeInfos) }
			, theWgtFacing{ weightFacing(theDotFacing) }
				// Average distance of other pixel to own edge line seg
			, theLineGapDist{ lineGapDist(theNdx1, theNdx2, edgeInfos) }
			, theWgtLineGap
				{ weightCollinear(theLineGapDist, lineGapSigma, theDotFacing) }
		{ }

		//! Pseudo-probability of edges facing each other
		inline
		double
		weightFacing
			() const
		{
			double wgt{ 0. };
			if (0. < theDotFacing)
			{
				wgt = theWgtFacing;
			}
			return wgt;
		}

		//! Pseudo-probability of being collinear (based on ctor lineGapSigma)
		inline
		double const &
		weightCollinear
			() const
		{
			return theWgtLineGap;
		}

		/*! \brief Pseudo-probability edgels are on opposite radial edges.
		 *
		 * Assign pseudo probability of radial edge pair requiring:
		 * \arg Oppositing gradient directions (for +/- radius)
		 * \arg Approx collinearity of spots perp to edge grads
		 */
		inline
		double
		weightRadialPair
			() const
		{
			return (weightFacing() * weightCollinear());
		}

		//! Composite gradient associated edgel pair (positive with ndx1)
		inline
		img::Grad
		pairDir
			( std::vector<EdgeInfo> const & edgeInfos
			) const
		{
			EdgeInfo const & ei1 = edgeInfos[theNdx1];
			EdgeInfo const & ei2 = edgeInfos[theNdx2];
			img::Grad const dir1{ ei1.edgeDirection() };
			img::Grad const dir2{ ei2.edgeDirection() };
			//
			// NOTE: treat dir1 as positive direction and negate dir2
			//
			img::Grad const sum{ dir1 - dir2 };
			img::Grad const mean{ direction(sum) };
			return mean;
		}

		//! Angle associated with dir
		inline
		double
		angle
			( img::Grad const & dir
			) const
		{
			return ang::atan2(dir[1], dir[0]);
		}

	}; // EdgePair



} // [sig]

} // [quadloco]

