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
 * \brief Declarations for quadloco::prb::Histo namespace
 *
 */


#include "valSpan.hpp"

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>


namespace quadloco
{

namespace prb
{

	/*! \brief Simple histogram accumulation and peak finding (no-wrap)
	 *
	 * Example
	 * \snippet test/test_prbHisto.cpp DoxyExample01
	 */
	class Histo
	{
		std::vector<double> theWeights;
		val::Span const theValueSpan;

		// local/cached values
		val::Span const theIndexSpan;
		double const theBinSize;
		double const theNdxPerVal;
		double theWgtSum{ 0. };

	private:

		//! Accumulation weights
		inline
		double const &
		weightAtIndex
			( std::size_t const & binNdx
			) const
		{
			return theWeights[binNdx];
		}

		//! Accumulation weights
		inline
		std::vector<double> const &
		weights
			() const
		{
			return theWeights;
		}

		//! Sum over all weights()
		inline
		double
		weightSum
			() const
		{
			return theWgtSum;
			/*
			double const sumWgts
				{ std::accumulate(theWeights.cbegin(), theWeights.cend(), 0.) };
			return sumWgts;
			*/
		}

	public:

		/*! \brief Construct histrogram with numBins over specified data range.
		 *
		 * For "N" bins,
		 * \arg Bin size is (valueSpan.magnitude() / N)
		 * \arg Bin[0] starts at valueSpan.min()
		 * \arg Bin[N-1] ends just *BEFORE* valueSpan.max()
		 */
		inline
		explicit
		Histo
			( std::size_t const & numBins
				//!< Specify resolution of histogram bin array
			, val::Span const & valueSpan
				//!< Half-open range of values [min, max) to fill array
			)
			: theWeights(numBins, 0.) // allocate and zero
			, theValueSpan{ valueSpan }
			//
			, theIndexSpan{ 0., static_cast<double>(numBins) }
			, theBinSize
				{ theValueSpan.magnitude() / theIndexSpan.magnitude() }
			, theNdxPerVal{ 1. / theBinSize }
			, theWgtSum{ 0. }
		{ }

		//! True if constructed with sane values
		inline
		bool
		isValid
			() const
		{
			return
				(  theIndexSpan.isValid()
				&& theValueSpan.isValid()
				&& (0. < theBinSize)
				);
		}

		//! Incorporate a gaussian like (unnormalized) 'bump' into bins
		inline
		void
		addValue
			( double const & value
			, double const & valueSigma
			, double const & nSigmaClip = 2.5
			)
		{
			if ( (! (value < theValueSpan.min()))
			  &&    (value < theValueSpan.max())
			   )
			{
				double const frac{ theValueSpan.fractionAtValue(value) };
				double const dNdx{ theIndexSpan.valueAtFraction(frac) };
				double dBin{ 0. };
				double const binFrac{ std::modf(dNdx, &dBin) };
				int const hitNdx{ static_cast<int>(dBin) };
				double const maxDeltaVal{ nSigmaClip * valueSigma };
				double const maxDeltaNdx{ theNdxPerVal * maxDeltaVal };
				int const nDelta{ static_cast<int>(std::ceil(maxDeltaNdx)) };

				/*
				std::cout << "       dBin: " << dBin << '\n';
				std::cout << "     hitNdx: " << hitNdx << '\n';
				std::cout << "    binFrac: " << binFrac << '\n';
				std::cout << "maxDeltaVal: " << maxDeltaVal << '\n';
				std::cout << "maxDeltaNdx: " << maxDeltaNdx << '\n';
				std::cout << " valueSigma: " << valueSigma << '\n';
				*/

				for (int relNdx{-nDelta} ; (! (nDelta < relNdx)) ; ++relNdx)
				{
					int binNdx{ hitNdx + relNdx };
					if (binNdx < theWeights.size())
					{
						double const valueDiff{ value - valueAtIndex(binNdx) };
						double const probArg{ valueDiff / valueSigma };
						double const probWgt{ std::exp(-probArg*probArg) };
						theWeights[binNdx] += probWgt;
						theWgtSum += probWgt;

						/*
						std::cout
							<< "relNdx: "
								<< std::setw(4u) << relNdx
							<< " binNdx: "
								<< std::setw(4u) << binNdx
							<< " valueDiff: "
								<< std::setw(9u) << std::fixed << valueDiff
							<< " probArg: "
								<< std::setw(9u) << std::fixed << probArg
							<< " probWgt: "
								<< std::setw(9u) << std::fixed << probWgt
							<< '\n';
						*/

					}

				}

			}
		}

		//! Number of bins in histogram accumulation array
		inline
		std::size_t
		size
			() const
		{
			return theWeights.size();
		}

		//! Data value associated with (start of) binNdx
		inline
		double
		valueAtIndex
			( std::size_t const & binNdx
			) const
		{
			double const delta{ theBinSize * (double)binNdx };
			return (theValueSpan.min() + delta);
		}

		//! Probability associated with (start of) binNdx
		inline
		double
		probabilityAtIndex
			( std::size_t const & binNdx
			) const
		{
			double prob{ std::numeric_limits<double>::quiet_NaN() };
			if (0. < theWgtSum)
			{
				prob = (1./theWgtSum) * theWeights[binNdx];
			}
			return prob;
		}

		/* TODO - comment/test if needed
		//! Values of bins (corresponding with probabilities())
		inline
		std::vector<double>
		values
			() const
		{
			std::vector<double> vals;
			std::size_t const numElem{ size() };
			vals.reserve(numElem);
			for (std::size_t binNdx{0u} ; binNdx < numElem ; ++binNdx)
			{
				vals.emplace_back(valueAtIndex(binNdx));
			}
			return vals;
		}
		*/

		//! Accumulated probabilities (corresponding with values())
		inline
		std::vector<double>
		probabilities
			() const
		{
			std::vector<double> probs;
			probs.reserve(theWeights.size());
			double const sum{ weightSum() };
			std::transform
				( theWeights.cbegin(), theWeights.cend()
				, std::inserter(probs, probs.begin())
				, [& sum] (double const & wgt) { return (wgt / sum); }
				);
			return probs;
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
				<< "size: " << theWeights.size()
				<< '\n'
				<< "theIndexSpan: " << theIndexSpan
				<< '\n'
				<< "theValueSpan: " << theValueSpan
				<< '\n'
				<< "  theBinSize: " << fixed(theBinSize, 3u, 3u)
				<< '\n'
				<< "theNdxPerVal: " << fixed(theNdxPerVal, 3u, 3u)
				;
			return oss.str();
		}

		//! Description of the contents of this instance
		inline
		std::string
		infoStringContents
			( std::string const & title = {}
			) const
		{
			std::ostringstream oss;
			double const sumWgts{ weightSum() };
			double norm{ std::numeric_limits<double>::quiet_NaN() };
			if (0. < sumWgts)
			{
				norm = (1. / sumWgts);
			}
			oss << infoString(title);
			oss << "\n# binNdx, valueAtIndex, weights, binProb";
			for (std::size_t bNdx{0u} ; bNdx < theWeights.size() ; ++bNdx)
			{
				using engabra::g3::io::fixed;
				oss << '\n';
				oss
					<< std::setw(5u) << bNdx
					<< ' ' << fixed(valueAtIndex(bNdx))
					<< ' ' << fixed(weightAtIndex(bNdx))
					<< ' ' << fixed(norm * weightAtIndex(bNdx))
					;
			}
			return oss.str();
		}

	}; // Histo


} // [prb]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::prb::Histo const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::prb::Histo const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

