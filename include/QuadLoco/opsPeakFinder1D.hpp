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
 * \brief Declarations for quadloco::ops::PeakFinder1D
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
	struct PeakFinder1D
	{
		//! Collection of collections each of which contains indices in a peak
		std::vector<std::vector<std::size_t> > const thePeakNdxGrps{};

		/*! \brief Is input domain circular (wrap around) or linear (no wrap)
		 *
		 * Note that Linear domain treats values just before and just after
		 * the data array as having 0 values. E.g. a first element value
		 * of data[0]=2 is considered a Rise, and data[0]=-1 is considered
		 * a Drop.
		 */
		enum class DataDomain
		{
			  Linear //!< Input data domain is finite and unwrapped
			, Circle //!< Input data domain wraps around (e.g. angles)
		};

		//! Data value transition type (current pixel w.r.t. prevous pixel)
		enum class Change
		{
			  Drop = 0
			, Flat = 1
			, Rise = 2

		}; // Change

		//! \brief Track data index and Change from previous index to this one.
		struct NdxChange
		{
			//! The index at which Change is evaluated
			std::size_t theNdx;
			//! The change from (theNdx-1) until (theNdx).
			Change theChange;

		}; // NdxChange

		//! Name for Change enum values
		inline
		static
		std::string const &
		stringFor
			( Change const & change
			)
		{
			static std::array<std::string, 3u> const changeNames
				{ "d" //"v"//.Drop"
				, "s" //"-"//.same"
				, "u" //"^"//.Rise"
				};
			return changeNames[(std::size_t)change];
		}

#if 0
// TODO Assumes Circular wrap around!!
		//! Convert data stream into stream of value transition changes
		template <typename FwdIter>
		inline
		static
		std::vector<NdxChange>
		ndxChangesFor
			( FwdIter const & itBeg
			, FwdIter const & itEnd
			)
		{
			std::vector<NdxChange> ndxChanges;

			std::size_t const numElem
				{ (std::size_t)std::distance(itBeg, itEnd) };

			if (0u < numElem)
			{
				std::size_t const numLast{ numElem - 1u };
				ndxChanges.reserve(numElem);

				FwdIter itPrev{ itBeg + numLast };
				FwdIter itCurr{ itBeg };
				for (std::size_t nn{0u} ; nn < numElem ; ++nn)
				{
					using Type = typename FwdIter::value_type;
					Type const & valPrev = *itPrev;
					Type const & valCurr = *itCurr;

					Change change{ Change::Flat };
					if (valPrev < valCurr)
					{
						change = Change::Rise;
					}
					else
					if (valCurr < valPrev)
					{
						change = Change::Drop;
					}
					ndxChanges.emplace_back(NdxChange{ nn, change });

					// increment iterators and wrap at end
					++itPrev;
					++itCurr;
					if (itEnd == itPrev) { itPrev = itBeg; }
					if (itEnd == itCurr) { itCurr = itBeg; }
				}

			} // (1u < numElem)

			return ndxChanges;
		}
#endif

		//! Change associated with value transition from previous to current
		template <typename Type>
		inline
		static
		Change
		changeFor
			( Type const & valPrev
			, Type const & valCurr
			)
		{
			Change change{ Change::Flat };
			if (valPrev < valCurr)
			{
				change = Change::Rise;
			}
			else
			if (valCurr < valPrev)
			{
				change = Change::Drop;
			}
			return change;
		}

		//! Determine value transition change at ndxCurr from itBeg
		template <typename FwdIter>
		inline
		static
		Change
		changeForIndexCircle
			( std::size_t const & ndxCurr
			, std::size_t const & numElem
			, FwdIter const & itBeg
			)
		{
			FwdIter const itCurr{ itBeg + ndxCurr };
			std::size_t const ndxPrev{ (ndxCurr + (numElem - 1u)) % numElem };
			FwdIter const itPrev{ itBeg + ndxPrev };

			using Type = typename FwdIter::value_type;
			Type const & valPrev = *itPrev;
			Type const & valCurr = *itCurr;

			return changeFor(valPrev, valCurr);
		}

		//! Determine value transition change at ndxCurr from itBeg
		template <typename FwdIter>
		inline
		static
		Change
		changeForIndexLinear
			( std::size_t const & ndxCurr
			, std::size_t const & numElem
			, FwdIter const & itBeg
			)
		{
			Change change{ Change::Flat };
			if ((0u < ndxCurr) && ((ndxCurr + 1u) < numElem))
			{
				change = changeForIndexCircle(ndxCurr, numElem, itBeg);
			}
			else
			{
				FwdIter const itCurr{ itBeg + ndxCurr };
				using Type = typename FwdIter::value_type;
				Type const & valCurr = *itCurr;
				if (0u == ndxCurr)
				{
					Type const valPrev{ 0 };
					change = changeFor(valPrev, valCurr);
				}
				else
				if (numElem == (ndxCurr + 1u))
				{
					Type const valNext{ 0 };
					change = changeFor(valCurr, valNext);
				}
			}
			return change;
		}

		//! Determine value transition change at ndxCurr from itBeg
		template <typename FwdIter>
		inline
		static
		Change
		changeForIndex
			( std::size_t const & ndxCurr
			, std::size_t const & numElem
			, FwdIter const & itBeg
			, DataDomain const & dataDomain
			)
		{
			if (DataDomain::Circle == dataDomain)
			{
				return changeForIndexCircle(ndxCurr, numElem, itBeg);
			}
			else
			// if (DataDomain::Linear == dataDomain)
			{
				return changeForIndexLinear(ndxCurr, numElem, itBeg);
			}
		}

		//! Construct collections of peaks from data values
		template <typename FwdIter>
		inline
		explicit
		PeakFinder1D
			( FwdIter const & itBeg
			, FwdIter const & itEnd
			, DataDomain const & dataDomain = DataDomain::Circle
			)
			: thePeakNdxGrps{ peakIndexGroups(itBeg, itEnd, dataDomain) }
		{ }

		//! Location of peaks (midpoint of flat-top peaks)
		inline
		std::vector<std::size_t>
		peakIndices
			() const
		{
			std::vector<std::size_t> peakNdxs;
			peakNdxs.reserve(thePeakNdxGrps.size());
			for (std::vector<std::vector<std::size_t> >::const_iterator
				itR{thePeakNdxGrps.cbegin()}
				; thePeakNdxGrps.cend() != itR ; ++itR)
			{
				std::vector<std::size_t> const & peakNdxGrp = *itR;
				// NOTE: Indices in group are reversed (largest to smallest)
				//       For lowest index in even sizes, pick back-most one
				std::size_t const midNdx{ peakNdxGrp.size() / 2u };
				peakNdxs.emplace_back(peakNdxGrp[midNdx]);
			}
			return peakNdxs;
		}

		//! Function object to track indices involved in peaks
		struct PeakTracker
		{
			//! Collections of collections of indices that comprise a peak
			std::vector<std::vector<std::size_t> > thePeakNdxGrps{};
			//! Collection of indices which are current *candidate* for peak
			std::vector<std::size_t> theActiveNdxs{};
			//! True when current tracking state *might* possibly be on a peak
			bool theIsTracking{};

			//! Allocate space for tracking structures, set tracking state
			inline
			explicit
			PeakTracker
				( std::size_t const & numElem
				, bool const & trackingState = false
				)
				: thePeakNdxGrps{}
				, theActiveNdxs{}
				, theIsTracking{ trackingState }
			{
				thePeakNdxGrps.reserve(numElem);
				theActiveNdxs.reserve(numElem);
			}

			//! Mark begining of (potential) peak - enable tracking
			inline
			void
			beginPeakMaybe
				( std::size_t const & ndx
				)
			{
				theActiveNdxs.clear();
				theActiveNdxs.emplace_back(ndx);
				theIsTracking = true;
			}

			//! Mark end of peak - disable tracking
			inline
			void
			endPeak
				()
			{
				if (! theActiveNdxs.empty())
				{

					/*
					std::cout << "  endPeak: size:"
						<< ' ' << theActiveNdxs.size() << "  ndxs: ";
					for (std::size_t const & ndx : theActiveNdxs)
					{
						std::cout << ' ' << ndx;
					}
					std::cout << '\n';
					*/

					thePeakNdxGrps.emplace_back(theActiveNdxs);
					theActiveNdxs.clear();
				}
				theIsTracking = false;
			}

			//! True if current tracking top of (potential) peak
			inline
			bool
			isTracking
				() const
			{
				return theIsTracking;
			}

			//! start Ndx of current tracking peak - only valid if isTracking()
			inline
			std::size_t
			currPeakBegNdx
				() const
			{
				std::size_t begNdx{ std::numeric_limits<std::size_t>::max() };
				if (! theActiveNdxs.empty())
				{
					begNdx = theActiveNdxs.front();
				}
				return begNdx;
			}

			//! Consider ndx with change from data[ndx-1] to data[ndx]
			inline
			void
			consider
				( Change const & changePrevToCurr
				, std::size_t const & ndx
				)
			{
				if (Change::Rise == changePrevToCurr)
				{
					// not part of a peak, start of peak potentially follows
					beginPeakMaybe(ndx);
				}
				else
				if (Change::Drop == changePrevToCurr)
				{
					if (isTracking())
					{
						endPeak();
					}
					// else // continue waiting for a rise
				}
				else // (Change::Flat == changePrevToCurr)
				{
					if (isTracking())
					{
						// on potential peak, save indices until confirm or not
						theActiveNdxs.emplace_back(ndx);
					}
					// else // after a peak, waiting for a rise
				}
			}

		}; // PeakTracker

		/*! \brief Return collections of indices associated with data peaks.
		 *
		 * Each element in the return collection is a groups of indices
		 * associated with an individual peak in the overall data. The
		 * indices within each group span the width of a spread out
		 * (e.g. flat-top) peak.
		 */
		template <typename FwdIter>
		inline
		static
		std::vector<std::vector<std::size_t> >
		peakIndexGroups
			( FwdIter const & itBeg
			, FwdIter const & itEnd
			, DataDomain const & dataDomain = DataDomain::Circle
			)
		{
			std::vector<std::vector<std::size_t> > peakNdxGrps;
			std::size_t const numElem
				{ (std::size_t)std::distance(itBeg, itEnd) };

			if (0u < numElem)
			{
				std::size_t const numLast{ numElem - 1u };

				using Type = typename FwdIter::value_type;

				Type valuePrior{ 0 };
				Type valueAfter{ 0 };
				if (DataDomain::Circle == dataDomain)
				{
					valuePrior = *(itBeg + numLast);
					valueAfter = *itBeg;
				}

				PeakTracker tracker(numElem);

constexpr bool show{ false };
if (show)
{
std::cout << '\n';
std::cout << "valuePrior: " << valuePrior << '\n';
std::cout << "valueAfter: " << valueAfter << '\n';
}

				std::vector<std::size_t> currPeakNdxs;
				currPeakNdxs.reserve(numElem);

				// initial change (prior compared to curr(ndx=0);
				std::size_t ndx{0u};
				FwdIter const iterDawn{ itBeg + ndx };
				Change const changeDawn{ changeFor(valuePrior, *iterDawn) };
if (show)
{
std::cout << "ndx,change:"
	<< ' ' << "from:"
	<< ' ' << std::setw(2u) << std::fixed << valuePrior
	<< ' ' << "into:"
	<< ' ' << "data[" << ndx << "]:"
	<< ' ' << std::setw(2u) << std::fixed << *iterDawn
	<< ' ' << stringFor(changeDawn)
	<< '\n';
}

				tracker.consider(changeDawn, ndx);

				// intermediate changes
				for (ndx = 1u ; ndx < numElem ; ++ndx)
				{
					FwdIter const iterPrev{ itBeg + (ndx - 1u) };
					FwdIter const iterCurr{ itBeg + ndx };
					Type const & valuePrev = *iterPrev;
					Type const & valueCurr = *iterCurr;
					Change const changeCurr{ changeFor(valuePrev, valueCurr) };
if (show)
{
std::cout << "ndx,change:"
	<< ' ' << "from:"
	<< ' ' << std::setw(2u) << std::fixed << valuePrev
	<< ' ' << "into:"
	<< ' ' << "data[" << ndx << "]:"
	<< ' ' << std::setw(2u) << std::fixed << *iterCurr
	<< ' ' << stringFor(changeCurr)
	<< '\n';
}

					tracker.consider(changeCurr, ndx);
				}

				// ONE PAST - end condition change
				ndx = numElem;
				FwdIter const iterLast{ itBeg + numLast };
				Change const changeLast{ changeFor(*iterLast, valueAfter) };
				if (Change::Drop == changeLast)
				{
					tracker.endPeak();
				}
if (show)
{
std::cout << "ndx,change:"
	<< ' ' << "from:"
	<< ' ' << "data[" << ndx << "]:"
	<< ' ' << std::setw(2u) << std::fixed << *iterLast
	<< ' ' << "into:"
	<< ' ' << std::setw(2u) << std::fixed << valueAfter
	<< ' ' << stringFor(changeLast)
	<< '\n';
}

				// Wrap-around peak continuation
				if (Change::Drop == changeLast)
				{
					tracker.endPeak();
				}
				else
				if (Change::Rise == changeLast)
				{
					// tracker.beginPeakMaybe(); // no effect afterward
				}
				else
				if (Change::Flat == changeLast)
				{
					// for circular domain,
					// if peak is active at end, continue (re-)tracking
					// from begining of currently active peak
					if ( tracker.isTracking()
					  && (DataDomain::Circle == dataDomain)
					   )
					{
						std::size_t const begNdx{ tracker.currPeakBegNdx() };
						if (begNdx < std::numeric_limits<std::size_t>::max())
						{
							for (std::size_t ndx{0u} ; ndx < begNdx ; ++ndx)
							{
								std::size_t const ndxWrap
									{ (numElem - 1u + ndx) % numElem };

if (show)
{
std::cout << "check for wrap around\n";
std::cout << "iterPrev at: " << ndxWrap << '\n';
}

								FwdIter const iterPrev{ itBeg + ndxWrap };
								FwdIter const iterCurr{ itBeg + ndx };
								Type const & valuePrev = *iterPrev;
								Type const & valueCurr = *iterCurr;
								Change const changeCurr
									{ changeFor(valuePrev, valueCurr) };
if (show)
{
std::cout << "ndx,change:"
	<< ' ' << "from:"
	<< ' ' << std::setw(2u) << std::fixed << valuePrev
	<< ' ' << "into:"
	<< ' ' << "data[" << ndxWrap << "]:"
	<< ' ' << std::setw(2u) << std::fixed << *iterCurr
	<< ' ' << stringFor(changeCurr)
	<< '\n';
}

								tracker.consider(changeCurr, ndx);
								if (! tracker.isTracking())
								{
									break;
								}

							} // for(ndx)

						} // begNdx

					} // tracking DataDomain::Circle

				} // (Change::Flat == changeLast)

				peakNdxGrps = tracker.thePeakNdxGrps;

			} // (0 < numElem)

			/*
			for (std::vector<std::size_t> const & peakNdxGrp : peakNdxGrps)
			{
				std::cout << "@@ NdxGrp\n";
				for (std::size_t const & peakNdx : peakNdxGrp)
				{
					std::cout << "@@    peakNdx: " << peakNdx << '\n';
				}
			}
			*/

			return peakNdxGrps;
		}

	}; // PeakFinder1D


} // [ops]

} // [quadloco]

/*
				struct PeakTrack
				{
					std::vector<std::size_t> theCurrNdxs;

					inline void consider
						( Change const & change
						)
					{
					}

					inline void restart()
						{ theCurrNdxs.clear() };

					inline void addTo
						(std::vector<std::vector<std::size_t> > * const & ptAll
						)
					{
						ptAll->emplace_back(theCurrNdxs)
					}

				}; // PeakTrack

				PeakTrack ptRise{};
				PeakTrack ptDrop{};
*/
