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
 * \brief Declarations for quadloco::sig::PeakFinder
 *
 */


#include <algorithm>
#include <array>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <vector>


namespace quadloco
{

/*! \brief Stop level namespace for signal processing, quadloco::sig
 */
namespace sig
{

	//! Utility for finding inflection peaks in array of data
	struct PeakFinder
	{
		enum Flag
		{
			  Drop = 0
			, Flat = 1
			, Rise = 2

		}; // Flag

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

		//! Convert data stream into stream of flags
		template <typename FwdIter>
		inline
		static
		std::vector<Flag>
		flagsFor
			( FwdIter const & itBeg
			, FwdIter const & itEnd
			)
		{
			std::vector<Flag> flags;
			std::vector<NdxFlag> ndxFlags;

			std::size_t const numElem
				{ (std::size_t)std::distance(itBeg, itEnd) };

			std::vector<std::set<std::size_t> > peakNdxSets;
			peakNdxSets.reserve(numElem);

			if (1u < numElem)
			{
				std::size_t const numLast{ numElem - 1u };
				flags.resize(numElem);

				std::vector<Flag>::iterator itFlag{ flags.begin() };
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
					flags.emplace_back(flag);
					ndxFlags.emplace_back(NdxFlag{ nn, flag });

					// increment iterators and wrap at end
					++itPrev;
					++itCurr;
					if (itEnd == itPrev) { itPrev = itBeg; }
					if (itEnd == itCurr) { itCurr = itBeg; }
				}

				bool trackingPeak{ false };
				std::size_t ndxA{ std::numeric_limits<std::size_t>::max() };
				std::set<std::size_t> peakNdxs;
			//	std::string dnote;
			//	std::string unote;
			//	std::string tnote;
				for (std::size_t nn{0u} ; nn < numElem ; ++nn)
				{
					std::size_t const nnRev{ numLast - nn };

					NdxFlag const & ndxFlag = ndxFlags[nnRev];
					std::size_t const & ndx = ndxFlag.theNdx;
					Flag const & flag = ndxFlag.theFlag;

					if (trackingPeak)
					{
						peakNdxs.insert(ndx);
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
						peakNdxs.clear();
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
							peakNdxSets.emplace_back(peakNdxs);
							trackingPeak = false;
						}
						else
						{
							// spurious 'rise' encountered outside of peak
						//	unote = "u-end  ";
							peakNdxs.clear();
						}
					}

					/*
					tnote = "    ";
					if (trackingPeak)
					{
						tnote = "t-on";
					}
					std::cout
						<< "ndx: " << std::setw(3u) << ndxFlag.theNdx
						<< ' '
						<< "flag: " << stringFor(ndxFlag.theFlag)
						<< ' '
						<< dnote
						<< ' '
						<< tnote
						<< ' '
						<< unote
						<< '\n'
						;
					*/

				}

				for (std::set<std::size_t> const & peakNdxSet : peakNdxSets)
				{
					std::cout << "peakNdxSet: ";
					for (std::set<std::size_t>::const_iterator
						iter{peakNdxSet.cbegin()}
						; peakNdxSet.cend() != iter ; ++iter)
					{
						std::cout << ' ' << *iter;
					}
					std::cout << '\n';
				}

			} // (1u < numElem)

			return flags;
		}

	}; // PeakFinder


} // [sig]

} // [quadloco]

