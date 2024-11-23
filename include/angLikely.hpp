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
 * \brief Declarations for quadloco::ang::Likely
 *
 */


#include "ang.hpp"
#include "datRing.hpp"
#include "datSpot.hpp"
#include "prbGauss1D.hpp"
#include "sigPeakFinder.hpp"

#include <Engabra>

#include <cmath>
#include <sstream>
#include <string>
#include <vector>


namespace quadloco
{

namespace ang
{
	//! \brief Angle likelihood estimation
	struct Likely
	{
		dat::Ring const theRing{};
		std::vector<double> theBinSums{};

		//! Construct a default (null) instance
		inline
		explicit
		Likely
			() = default;

		//! Construct accumulation buffer with requested number of bins
		inline
		explicit
		Likely
			( std::size_t const & numAngBins
			)
			: theRing(numAngBins)
			, theBinSums(numAngBins, 0.)
		{ }

		//! True if this instance is valid (not null)
		inline
		bool
		isValid
			() const
		{
			return
				(  theRing.isValid()
				&& (0u < theBinSums.size())
				);
		}

		//! Number of bins in accumulation buffer.
		inline
		std::size_t
		size
			() const
		{
			return theBinSums.size();
		}

		//! Angular resolution of accumulation buffer (aka bin width)
		inline
		double
		angleDelta
			() const
		{
			return theRing.angleDelta();
		}

		//! Incorporate weighted angle value into ring buffer
		inline
		void
		add
			( double const & angle
				//!< Angle value at which to add weighted contributions
			, double const & weight = 1.
				//!< Overall weight with which to accumulate this angle
			, std::size_t const & halfBinSpread = 1u
				//!< Spread weighting over into this many bins on each side
			)
		{
			double const binDelta{ theRing.angleDelta() };
			static prb::Gauss1D const gauss(0., binDelta);

			// add largest weight into main bin
			std::size_t const ndxCurr{ theRing.indexFor(angle) };
			double const offset{ angle - theRing.angleAt(ndxCurr) };
			theBinSums[ndxCurr] += weight * gauss(offset);

			// add decreasing weights into (circularly) adjacent bins
			for (std::size_t dn{0u} ; dn < halfBinSpread ; ++dn)
			{
				double const angDelta{ binDelta * (double)dn };

				int const dnPos{ (int)dn };
				std::size_t const ndxPos
					{ theRing.indexRelativeTo(ndxCurr, dnPos) };
				theBinSums[ndxPos] += weight * gauss(offset + angDelta);

				int const dnNeg{ -dnPos };
				std::size_t const ndxNeg
					{ theRing.indexRelativeTo(ndxCurr, dnNeg) };
				theBinSums[ndxNeg] += weight * gauss(offset - angDelta);
			}
		}

		//! Local peaks in angle data buffer
		inline
		std::vector<std::size_t>
		indicesOfPeaks
			() const
		{
			sig::PeakFinder const peakFinder
				(theBinSums.cbegin(), theBinSums.cend());
			return peakFinder.peakIndices();
		}

		//! Angle at ndxCurr adjusted to reflect theBinSum of neighbors
		inline
		double
		weightedAngleAt
			( std::size_t const & ndxCurr
			) const
		{
			// compute weighted peak - self and two adjacent ones
			std::size_t const ndxPrev
				{ theRing.indexRelativeTo(ndxCurr, -1) };
			std::size_t const ndxNext
				{ theRing.indexRelativeTo(ndxCurr,  1) };

			// get angle and neighbor angle values
			double const anglePrev{ theRing.angleAt(ndxPrev) };
			double const angleCurr{ theRing.angleAt(ndxCurr) };
			double const angleNext{ theRing.angleAt(ndxNext) };

			// do math with vectors to avoid phase wrap problems
			dat::Spot const spotPrev
				{ std::cos(anglePrev), std::sin(anglePrev) };
			dat::Spot const spotCurr
				{ std::cos(angleCurr), std::sin(angleCurr) };
			dat::Spot const spotNext
				{ std::cos(angleNext), std::sin(angleNext) };

			// use accumulation buffer as weights
			double const wgtPrev{ theBinSums[ndxPrev] };
			double const wgtCurr{ theBinSums[ndxCurr] };
			double const wgtNext{ theBinSums[ndxNext] };

			// compute weighted average location
			dat::Spot const wSpotSum
				{ wgtPrev * spotPrev
				+ wgtCurr * spotCurr
				+ wgtNext * spotNext
				};
			double const wSum
				{ wgtPrev
				+ wgtCurr
				+ wgtNext
				};
			// if there's a peak here, at least one of the weights
			// must be greater than zero - so okay to divide
			dat::Spot const wSpot{ (1./wSum) * wSpotSum };

			// convert weighted spot location to angle
			double const wgtAngle{ ang::atan2(wSpot[1], wSpot[0]) };
			return wgtAngle;
		}

		//! Angles for local peaks (near middle for plateaus)
		inline
		std::vector<double>
		anglesOfPeaks
			() const
		{
			std::vector<double> peakAngles;
			std::vector<std::size_t> const ndxs{ indicesOfPeaks() };
			peakAngles.reserve(ndxs.size());
			for (std::size_t const & ndx : ndxs)
			{
				peakAngles.emplace_back(weightedAngleAt(ndx));
			}
			return peakAngles;
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
			oss
				<< "theRing: " << theRing
				<< ' '
				<< "binSize: " << size()
				;
			return oss.str();
		}

		//! Description of the buffer contents
		inline
		std::string
		infoStringContents
			( std::string const & title = {}
			) const
		{
			std::ostringstream oss;
			oss << infoString(title);
			for (std::size_t nbin{0u} ; nbin < theBinSums.size() ; ++nbin)
			{
				using engabra::g3::io::fixed;
				oss
					<< '\n'
					<< std::setw(3u) << nbin
					<< ' ' << fixed(theRing.angleAt(nbin), 6u, 6u)
					<< ' ' << fixed(theBinSums[nbin], 6u, 6u)
					;
			}
			return oss.str();
		}

	}; // Likely


} // [ang]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::ang::Likely const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::ang::Likely const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

