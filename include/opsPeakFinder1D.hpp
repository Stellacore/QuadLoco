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
		std::vector<std::vector<std::size_t> > const thePeakNdxGrps{};

// TODO - explain Linear assumes 0 values on each end
		//! Treat input domain is circular (wrap around) or linear (no wrap)
		enum DataDomain
		{
			  Linear //!< Input data domain is finite and unwrapped
			, Circle //!< Input data domain wraps around (e.g. angles)
		};

		//! Data value transition type (current pixel w.r.t. prevous pixel)
		enum Change
		{
			  Drop = 0
			, Flat = 1
			, Rise = 2

		}; // Change

		//! 
		struct NdxChange
		{
			std::size_t theNdx;
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

		//! Convert data stream into stream of value transition changes
// TODO Assumes Circular!!
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

					Change change{ Flat };
					if (valPrev < valCurr)
					{
						change = Rise;
					}
					else
					if (valCurr < valPrev)
					{
						change = Drop;
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

		//! Change associated with transition of values from previous to current
		template <typename Type>
		inline
		static
		Change
		changeFor
			( Type const & valPrev
			, Type const & valCurr
			)
		{
			Change change{ Flat };
			if (valPrev < valCurr)
			{
				change = Rise;
			}
			else
			if (valCurr < valPrev)
			{
				change = Drop;
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
			Change change{ Flat };
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
			if (Circle == dataDomain)
			{
				return changeForIndexCircle(ndxCurr, numElem, itBeg);
			}
			else
			// if (Linear == dataDomain)
			{
				return changeForIndexLinear(ndxCurr, numElem, itBeg);
			}
		}

#if 0
		/*! Return collections of indices associated with data peaks.
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
			, DataDomain const & dataDomain = Circle
			)
		{
			std::vector<std::vector<std::size_t> > peakNdxGrps;
			std::size_t const numElem
				{ (std::size_t)std::distance(itBeg, itEnd) };

std::cout << "\npeakIndexGroups:\n";
std::cout << "dataDomain: " << dataDomain << '\n';

			if (0u < numElem)
			{
				std::size_t const numLast{ numElem - 1u };

				// allocate space for index groups - one per possible peak
				peakNdxGrps.reserve(numElem); // an impossible worst case

				// set end point changes
				Change const change0
					{ changeForIndex(0, numElem, itBeg, dataDomain) };
				Change const changeN
					{ changeForIndex(numLast, numElem, itBeg, dataDomain) };
				// set initial tracking state
				bool trackingPeak{ (Drop == change0) };

				// indices for next upcomming peak
				std::vector<std::size_t> currPeakNdxs;
				currPeakNdxs.reserve(numElem); // worst case

std::cout << " change0: " << stringFor(change0) << '\n';
std::cout << " changeN: " << stringFor(changeN) << '\n';

				// Traverse data backward from end to begining.
				// Search for 'drop' features that signify the end of a peak.

			/**/	std::string dnote;
			/**/	std::string unote;
			/**/	std::string tnote;
				for (std::size_t nn{0u} ; nn < numElem ; ++nn)
				{
					std::size_t const nnRev{ numLast - nn };
					std::size_t const & ndx = nnRev;

					// NOTE: Could compute transition changes here as needed.
					Change const change
						{ changeForIndex(ndx, numElem, itBeg, dataDomain) };

					if (trackingPeak)
					{
						currPeakNdxs.emplace_back(ndx);
					}

					// when hitting drop (from reverse direction) is
					// candidate start of peak (until an 'up' unless
					// first hitting another drop)
				/**/	dnote = "       ";
					if (Drop == change)
					{
						// start MAYBE peak tracking (pending an up encounter)
					/**/	dnote = "d-start";
						trackingPeak = true;
						currPeakNdxs.clear();
					}

					// hitting a rise signals the (reverse direction) end
					// of peak when tracking is active.
				/**/	unote = "       ";
					if (Rise == change)
					{
						if (trackingPeak)
						{
							// all currently tracked indices are part of peak
						/**/	unote = "u-PEAK ";
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

					tnote = "    ";
					if (trackingPeak)
					{
						tnote = "t-on";
					}
					std::cout
						<< "ndx: " << std::setw(3u) << ndx
						<< ' ' << "change: " << stringFor(change)
						<< ' ' << dnote
						<< ' ' << tnote
						<< ' ' << unote
						<< '\n'
						;
					/*
					*/

				}
			}

			return peakNdxGrps;
		}
#endif

		//! Construct collections of peaks from data values
		template <typename FwdIter>
		inline
		explicit
		PeakFinder1D
			( FwdIter const & itBeg
			, FwdIter const & itEnd
			, DataDomain const & dataDomain = Circle
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
			std::vector<std::vector<std::size_t> > thePeakNdxGrps{};
			std::vector<std::size_t> theActiveNdxs{};
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
				if (Rise == changePrevToCurr)
				{
					// not part of a peak, start of peak potentially follows
					beginPeakMaybe(ndx);
				}
				else
				if (Drop == changePrevToCurr)
				{
					if (isTracking())
					{
						endPeak();
					}
					// else // continue waiting for a rise
				}
				else // (Flat == changePrevToCurr)
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

		/*! Return collections of indices associated with data peaks.
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
			, DataDomain const & dataDomain = Circle
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
				if (Circle == dataDomain)
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
				if (Drop == changeLast)
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
				if (Drop == changeLast)
				{
					tracker.endPeak();
				}
				else
				if (Rise == changeLast)
				{
					// tracker.beginPeakMaybe(); // no effect afterward
				}
				else
				if (Flat == changeLast)
				{
					// for circular domain,
					// if peak is active at end, continue (re-)tracking
					// from begining of currently active peak
					if (tracker.isTracking() && (Circle == dataDomain))
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

					} // tracking Circle

				} // (Flat == changeLast)

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
