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
 * \brief Declarations for quadloco::ops::AngleTracker
 *
 */


#include "QuadLoco/ang.hpp"
#include "QuadLoco/angRing.hpp"
#include "QuadLoco/opsPeakFinder1D.hpp"
#include "QuadLoco/prbGauss1D.hpp"

#include <Engabra>

#include <cmath>
#include <sstream>
#include <string>
#include <vector>


namespace quadloco
{

namespace ops
{
	/*! \brief Angle likelihood estimation via circular histogram management.
	 *
	 * Construct an instance with a specified number of angular
	 * quantiziation bins.
	 *
	 * Use the consider() function to incorporate weighted angle values into
	 * the histogram.
	 *
	 * Once accumulations are complete, the indicesOfPeaks() and
	 * anglesOfPeaks() methods report local peaks in the (then current)
	 * histogram.
	 */
	class AngleTracker
	{
		//! Provide angle/index relationship (with wrap around).
		ang::Ring const theRing{};

		//! Histogram of values - with indices matching those in #theRing.
		std::vector<double> theBinSums{};

		//! Cached values - current sum of all theBinSums
		double theTotalSum{ 0. };

	public:

		//! Construct a default (null) instance
		inline
		explicit
		AngleTracker
			() = default;

		//! Construct accumulation buffer with requested number of bins
		inline
		explicit
		AngleTracker
			( std::size_t const & numAngBins
			)
			: theRing(numAngBins)
			, theBinSums(theRing.size(), 0.)
			, theTotalSum{ 0. }
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

		//! Underlying circular ring buffer geometry
		inline
		ang::Ring const &
		angRing
			() const
		{
			return theRing;
		}

		//! Number of bins in accumulation buffer.
		inline
		std::size_t
		size
			() const
		{
			return theBinSums.size();
		}

		//! Angle associated with (start of) accumulation ndx-th bin
		inline
		double
		angleAtIndex
			( std::size_t const & ndx
			) const
		{
			return theRing.angleAtIndex(ndx);
		}

		//! Relative value of ndx-th accumulation buffer bin
		inline
		double
		probAtIndex
			( std::size_t const & ndx
			) const
		{
			double prob{ std::numeric_limits<double>::quiet_NaN() };
			if (std::numeric_limits<double>::epsilon() < theTotalSum)
			{
				if (ndx < theBinSums.size())
				{
					prob = (theBinSums[ndx] / theTotalSum);
				}
			}
			return prob;
		}

		//! Relative histogram value (probability) of histogram at this angle
		inline
		double
		probAtAngle
			( double const & angle
			) const
		{
			std::size_t const ndxCurr{ theRing.indexFor(angle) };
			return probAtIndex(ndxCurr);
		}

		/*! \brief Incorporate weighted angle probability density function.
		 *
		 * This function adds a (pseudo) probability density into the
		 * accumulation array.
		 *
		 * The density function is an (unnormalized) Guassian with mean
		 * equal to the angle value and with standard deviation equal to
		 * the halfBinSpread argument. The overall (Gaussian) function
		 * has max amplitied equal to the provided weight value.
		 *
		 * E.g.. adds a values into adjacement bins (da) with function
		 * of the form:
		 * \arg f(da) = weight * std::exp(-sq((angle-da) / halfBinSpread))
		 */
		inline
		void
		consider
			( double const & angle
				//!< Angle value at which to add weighted contributions
			, double const & weight = 1.
				//!< Overall weight with which to accumulate this angle
			, std::size_t const & halfBinSpread = 1u
				//!< Spread weighting over into this many bins on each side
			)
		{
			if (engabra::g3::isValid(angle))
			{
				double const binDelta{ theRing.angleDelta() };
				static prb::Gauss1D const gauss(0., binDelta);

				// add largest weight into main bin
				std::size_t const ndxCurr{ theRing.indexFor(angle) };
				double const offset{ angle - angleAtIndex(ndxCurr) };
				double const dSum{ weight * gauss(offset) };
				theBinSums[ndxCurr] += dSum;
				theTotalSum += dSum;

				// add decreasing weights into (circularly) adjacent bins
				for (std::size_t dn{1u} ; dn < halfBinSpread ; ++dn)
				{
					double const angDelta{ binDelta * (double)dn };

					int const dnPos{ (int)dn };
					std::size_t const ndxPos
						{ theRing.indexRelativeTo(ndxCurr, dnPos) };
					double const dSumPos{ weight * gauss(offset + angDelta) };
					theBinSums[ndxPos] += dSumPos;
					theTotalSum += dSumPos;

					int const dnNeg{ -dnPos };
					std::size_t const ndxNeg
						{ theRing.indexRelativeTo(ndxCurr, dnNeg) };
					double const dSumNeg{ weight * gauss(offset - angDelta) };
					theBinSums[ndxNeg] += dSumNeg;
					theTotalSum += dSumNeg;
				}
			}
		}

		//! Peak finder for exploring peaks in angle accumulation buffer
		inline
		PeakFinder1D
		peakFinder1D
			() const
		{
			return PeakFinder1D
				( theBinSums.cbegin()
				, theBinSums.cend()
				, PeakFinder1D::DataDomain::Circle
				);
		}

		//! Indices for local peaks (near middle for plateaus)
		inline
		std::vector<std::size_t>
		indicesOfPeaks
			() const
		{
			return peakFinder1D().peakIndices();
		}

		//! Angles associated with ndxs
		inline
		std::vector<double>
		anglesFor
			( std::vector<std::size_t> const & ndxs
			) const
		{
			std::vector<double> angles;
			angles.reserve(ndxs.size());
			for (std::size_t const & ndx : ndxs)
			{
				// return (start of)bin angle associated with ndx
				angles.emplace_back(angleAtIndex(ndx));
			}
			return angles;
		}

		//! Probabilities associated with ndxs
		inline
		std::vector<double>
		probsFor
			( std::vector<std::size_t> const & ndxs
			) const
		{
			std::vector<double> probs;
			probs.reserve(ndxs.size());
			for (std::size_t const & ndx : ndxs)
			{
				// return (start of)bin angle associated with ndx
				probs.emplace_back(probAtIndex(ndx));
			}
			return probs;
		}

		//! Angles for local peaks (near middle for plateaus)
		inline
		std::vector<double>
		anglesOfPeaks
			() const
		{
			return anglesFor(indicesOfPeaks());
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
			oss << "\n# binNdx  angle  binSum  probability";
			for (std::size_t nbin{0u} ; nbin < theBinSums.size() ; ++nbin)
			{
				using engabra::g3::io::fixed;
				oss
					<< '\n'
					<< std::setw(3u) << nbin
					<< ' ' << fixed(angleAtIndex(nbin), 6u, 6u)
					<< ' ' << fixed(theBinSums[nbin], 6u, 6u)
					<< ' ' << fixed(probAtIndex(nbin), 6u, 6u)
					;
			}
			return oss.str();
		}

	}; // AngleTracker


} // [ops]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::ops::AngleTracker const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::ops::AngleTracker const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

