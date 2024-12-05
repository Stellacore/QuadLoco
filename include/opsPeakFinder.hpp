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
 * \brief Declarations for quadloco::ops::PeakFinder
 *
 */


#include <algorithm>
#include <array>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>


namespace quadloco
{

/*! \brief Stop level namespace for signal processing, quadloco::sig
 */
namespace ops
{

	//! Utility for finding inflection peaks in array of data
	struct PeakFinder
	{
		std::vector<std::vector<std::size_t> > const thePeakNdxGrps{};

		//! Data value transition type (current pixel w.r.t. prevous pixel)
		enum Flag
		{
			  Drop = 0
			, Flat = 1
			, Rise = 2

		}; // Flag

		//! 
		struct NdxFlag
		{
			std::size_t theNdx;
			Flag theFlag;

		}; // NdxFlag

		//! Name for flag enum values
		inline
		static
		std::string const &
		stringFor
			( Flag const & flag
			)
		{
			static std::array<std::string, 3u> const flagNames
				{ "d" //"v"//.Drop"
				, "s" //"-"//.same"
				, "u" //"^"//.Rise"
				};
			return flagNames[(std::size_t)flag];
		}

		//! Convert data stream into stream of value transition flags
		template <typename FwdIter>
		inline
		static
		std::vector<NdxFlag>
		ndxFlagsFor
			( FwdIter const & itBeg
			, FwdIter const & itEnd
			)
		{
			std::vector<NdxFlag> ndxFlags;

			std::size_t const numElem
				{ (std::size_t)std::distance(itBeg, itEnd) };

			if (0u < numElem)
			{
				std::size_t const numLast{ numElem - 1u };
				ndxFlags.reserve(numElem);

				FwdIter itPrev{ itBeg + numLast };
				FwdIter itCurr{ itBeg };
				for (std::size_t nn{0u} ; nn < numElem ; ++nn)
				{
					using Type = FwdIter::value_type;
					Type const & valPrev = *itPrev;
					Type const & valCurr = *itCurr;

					Flag flag{ Flat };
					if (valPrev < valCurr)
					{
						flag = Rise;
					}
					else
					if (valCurr < valPrev)
					{
						flag = Drop;
					}
					ndxFlags.emplace_back(NdxFlag{ nn, flag });

					// increment iterators and wrap at end
					++itPrev;
					++itCurr;
					if (itEnd == itPrev) { itPrev = itBeg; }
					if (itEnd == itCurr) { itCurr = itBeg; }
				}

			} // (1u < numElem)

			return ndxFlags;
		}

		//! Determine value transition flag at ndxCurr from itBeg
		template <typename FwdIter>
		inline
		static
		Flag
		flagForIndex
			( std::size_t const & ndxCurr
			, std::size_t const & numElem
			, FwdIter const & itBeg
			)
		{
			FwdIter const itCurr{ itBeg + ndxCurr };
			std::size_t const ndxPrev{ (ndxCurr + (numElem - 1u)) % numElem };
			FwdIter const itPrev{ itBeg + ndxPrev };

			using Type = FwdIter::value_type;
			Type const & valPrev = *itPrev;
			Type const & valCurr = *itCurr;

			Flag flag{ Flat };
			if (valPrev < valCurr)
			{
				flag = Rise;
			}
			else
			if (valCurr < valPrev)
			{
				flag = Drop;
			}

			return flag;
		}

		//! Return collections of indices associated with data peaks
		template <typename FwdIter>
		inline
		static
		std::vector<std::vector<std::size_t> >
		peakIndexGroups
			( FwdIter const & itBeg
			, FwdIter const & itEnd
			)
		{
			std::vector<std::vector<std::size_t> > peakNdxGrps;
			std::size_t const numElem
				{ (std::size_t)std::distance(itBeg, itEnd) };

			if (0u < numElem)
			{
				std::size_t const numLast{ numElem - 1u };

				// gather indices assocaited with peaks
				peakNdxGrps.reserve(numElem); // impossible worst case

				// set initial tracking flag state based on wrap around
				// condition from first element
				Flag const flag0{ flagForIndex(0, numElem, itBeg) };
				bool trackingPeak{ (Drop == flag0) };

				std::vector<std::size_t> currPeakNdxs;
			//	std::string dnote;
			//	std::string unote;
			//	std::string tnote;
				for (std::size_t nn{0u} ; nn < numElem ; ++nn)
				{
					std::size_t const nnRev{ numLast - nn };
					std::size_t const & ndx = nnRev;

					// NOTE: Could compute transition flags here as needed.
					Flag const flag{ flagForIndex(ndx, numElem, itBeg) };

					if (trackingPeak)
					{
						currPeakNdxs.emplace_back(ndx);
					}

					// when hitting drop (from reverse direction) is
					// candidate start of peak (until an 'up' unless
					// first hitting another drop)
				//	dnote = "       ";
					if (Drop == flag)
					{
						// start MAYBE peak tracking (pending an up encounter)
					//	dnote = "d-start";
						trackingPeak = true;
						currPeakNdxs.clear();
					}

					// hitting a rise signals the (reverse direction) end
					// of peak when tracking is active.
				//	unote = "       ";
					if (Rise == flag)
					{
						if (trackingPeak)
						{
							// all currently tracked indices are part of peak
						//	unote = "u-PEAK ";
							peakNdxGrps.emplace_back(currPeakNdxs);
							trackingPeak = false;
						}
						else
						{
							// spurious 'rise' encountered outside of peak
						//	unote = "u-end  ";
							currPeakNdxs.clear();
						}
					}

					/*
					tnote = "    ";
					if (trackingPeak)
					{
						tnote = "t-on";
					}
					std::cout
						<< "ndx: " << std::setw(3u) << ndx
						<< ' ' << "flag: " << stringFor(flag)
						<< ' ' << dnote
						<< ' ' << tnote
						<< ' ' << unote
						<< '\n'
						;
					*/

				}
			}

			return peakNdxGrps;
		}

		//! Construct collections of peaks from data values
		template <typename FwdIter>
		inline
		explicit
		PeakFinder
			( FwdIter const & itBeg
			, FwdIter const & itEnd
			)
			: thePeakNdxGrps{ peakIndexGroups(itBeg, itEnd) }
		{ }

		//! Location of peaks (midpoint of flat-top peaks)
		inline
		std::vector<std::size_t>
		peakIndices
			() const
		{
			std::vector<std::size_t> peakNdxs;
			peakNdxs.reserve(thePeakNdxGrps.size());
			for (std::vector<std::vector<std::size_t> >::const_reverse_iterator
				itR{thePeakNdxGrps.crbegin()}
				; thePeakNdxGrps.crend() != itR ; ++itR)
			{
				std::vector<std::size_t> const & peakNdxGrp = *itR;
				// NOTE: Indices in group are reversed (largest to smallest)
				//       For lowest index in even sizes, pick back-most one
				std::size_t const midNdx{ peakNdxGrp.size() / 2u };
				peakNdxs.emplace_back(peakNdxGrp[midNdx]);
			}
			return peakNdxs;
		}


	}; // PeakFinder


} // [ops]

} // [quadloco]

