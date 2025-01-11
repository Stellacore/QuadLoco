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
 * \brief Declarations for quadloco::ops::SymRing namespace
 *
 */


#include "rasRelRC.hpp"
#include "imgSpot.hpp"
#include "prbStats.hpp"
#include "rasGrid.hpp"
#include "rasRowCol.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <vector>


namespace quadloco
{

namespace ops
{
	//! Return the result of (val*val)
	template <typename Type>
	inline
	Type
	sq
		( Type const & val
		)
	{
		return (val*val);
	}


	/*! \brief Collection or RelRC instances that line close to a circle
	 *
	 * The RelRC instances represent row-offsets and column-offsets
	 * relative to a current filter center pixel. I.e., the returned
	 * collection of RelRC defines a circle relative to the filter center.
	 */
	inline
	std::vector<ras::RelRC>
	annularRelRCs
		( std::size_t const & halfSize
			/*!< Radius of annular filter to apply
			 *
			 * Values are associated with
			 * \arg 0:  4-hood of adjacent cells in cardinal directions
			 * \arg 1:  8-hood on a quarter turn rotated square through
			 *          digaonal neighbors (e.g. diamond on +/-2 max
			 *          indices)
			 * \arg 2:  12-hood on diamond +/-3 max indices
			 * \arg 3+: Irregular shape approaching *quantized* circle
			 *          as ringHalfSize value gets larger
			 */
		)
	{
		std::vector<ras::RelRC> relRCs{};

		// perimeter should be 2*pi*r < 7*r,
		// but allow for duplicates during initial compuation
		std::size_t const numPerim{ 2u * (7u * halfSize) };
		relRCs.reserve(numPerim);

		// generate row/col offsets within anulus
		constexpr double pi{ std::numbers::pi_v<double> };
		constexpr double piHalf{ .5 * pi };
		double const rad{ static_cast<double>(halfSize) + .5 };
		double const da{ .25 * pi / rad };

		// determine row/col values for first quadrant
		for (double angle{0.} ; !(piHalf < angle) ; angle += da)
		{
			img::Spot const spot
				{ rad*std::cos(angle), rad*std::sin(angle) };
			ras::RelRC const relRC
				{ static_cast<int>(std::floor(.5 + spot[0]))
				, static_cast<int>(std::floor(.5 + spot[1]))
				};
			if (relRCs.empty())
			{
				relRCs.emplace_back(relRC);
			}
			else
			if (! (relRC == relRCs.front()))
			{
				relRCs.emplace_back(relRC);
			}
		}

		// fill second quadrant with reverse symmetry
		using Iter = std::vector<ras::RelRC>::const_reverse_iterator;
		Iter const endQtr{ relRCs.crend() };
		for (Iter iter{relRCs.crbegin()} ; endQtr != iter ; ++iter)
		{
			ras::RelRC const & relRC = *iter;
			ras::RelRC const negRC{ -relRC.theRelRow,  relRC.theRelCol };
			relRCs.emplace_back(negRC);
		}

		// fill second half with half-turn symmetry
		std::size_t const halfEnd{ relRCs.size() - 1u };
		for (std::size_t nn{0u} ; nn < halfEnd ; ++nn)
		{
			ras::RelRC const & relRC = relRCs[nn];
			ras::RelRC const negRC{ -relRC.theRelRow, -relRC.theRelCol };
			relRCs.emplace_back(negRC);
		}

		// remove duplicate entries
		std::vector<ras::RelRC>::iterator const newEnd
			{ std::unique(relRCs.begin(), relRCs.end()) };
		relRCs.resize(std::distance(relRCs.begin(), newEnd));

		/*
		std::cout << "relRCs.size(): " << relRCs.size() << '\n';
		for (ras::RelRC const & relRC : relRCs)
		{
			std::cout << "relRC:"
				<< ' ' << relRC.theRelRow
				<< ' ' << relRC.theRelCol
				<< '\n';
		}
		*/

		return relRCs;
	}


	/*! \brief An annular ring symmetry filter
	 *
	 * The response at at given cell is based on the values of neighbor
	 * pixels defined by a symmetric (quantized) annular ring centered 
	 * on the evaluation cell. Ideally, the annulus would be a circle
	 * but quantization produces an irregular shape (diamonds and square
	 * for very small annulus radii).
	 *
	 * The response value is a pseudo-probability based on how well
	 * the source data values in the annulus exhibit both "high contrast"
	 * and "half-turn rotation symmetry" and the annular region contains
	 * a "Balance" of small and large values.
	 *
	 * Contributions in filter response (via operator()) include:
	 *
	 * \arg Balance: This is a threshhold criteria requiring the annular
	 *      region to have at least 'N' (diametrically opposite) pairs with
	 *      an average value above the theSrcMidValue and at least 'N' pairs
	 *      with an average value below that value. (N == theMinPosNeg)
	 *
	 * \arg High Contrast: The range betwee lo/hi values within the
	 *      annulus is used as a weight that modifies the symmetry
	 *      rotation pseudo-probability.
	 *
	 * \arg Half-Turn Symmetry: Values in one half of the annulus, when
	 *      considered in increasing angle order, should match the
	 *      values in the second half in that same order. The sum of
	 *      squared differences between the "first-half" and "second
	 *      half" provides a measure of dissimilarity. This is converted
	 *      to a pseudo-probability of sameness (via std::exp(-ssd*ssd)).
	 */
	struct SymRing
	{
		ras::Grid<float> const * const thePtSrc{ nullptr };

		float theSrcMidValue{};
		float theSrcFullRange{};

		int const theHalfFilterSize{};
		std::vector<ras::RelRC> theRelRCs{};
		std::size_t const theHalfRingSize{ 0u };

		static constexpr std::size_t theMinPosNeg{ 1u };

		//! \brief Construct to operate on ptSrc image.
		inline
		explicit
		SymRing
			( ras::Grid<float> const * const & ptSrc
				//!< Access source data grid (e.g. raw or smoothed image, etc)
			, prb::Stats<float> const & srcStats
				//!< Statistics on source data grid
			, std::size_t const & halfSize
				//!< Controls filter size \ref annularRelRCs()
			)
			: thePtSrc{ ptSrc }
			, theSrcMidValue{ .5f * (srcStats.max() + srcStats.min()) }
			, theSrcFullRange{ srcStats.range() }
			, theHalfFilterSize{ static_cast<int>(halfSize + 1u) }
			, theRelRCs{ annularRelRCs(halfSize) }
			, theHalfRingSize{ theRelRCs.size() / 2u }
		{ }

		//! Nominal "radial" size of annulus
		inline
		std::size_t
		halfSize
			() const
		{
			return static_cast<std::size_t>(theHalfFilterSize);
		}

		//! Nominal "diagonal" size of annulus
		inline
		std::size_t
		fullSize
			() const
		{
			return (2u*halfSize() + 1u);
		}

		//! \brief Track statistics for values in annular filter ring
		struct RingStats
		{
			// track contrast limits over annulus
			double theMin{ std::numeric_limits<double>::quiet_NaN() };
			double theMax{ std::numeric_limits<double>::quiet_NaN() };

			std::size_t theCount{ 0u };

			// track differences in radially opposite values
			double theSumSqDif{ 0.f };

			// track balance of +/- values in the annulus
			std::size_t theNumPos{ 0u };
			std::size_t theNumNeg{ 0u };

			//! True if there is at least one sample
			inline
			bool
			isValid
				() const
			{
				return (0u < theCount);
			}

			//! Consider pair of values on radial opposite sides of filter
			inline
			void
			consider
				( double const & delta1
				, double const & delta2
				)
			{
				// track annulus min/max
				if (! (0u < theCount))
				{
					theMin = std::min(delta1, delta2);
					theMax = std::max(delta1, delta2);
				}
				else
				{
					theMin = std::min(delta1, theMin);
					theMin = std::min(delta2, theMin);
					theMax = std::max(delta1, theMax);
					theMax = std::max(delta2, theMax);
				}

				// compute dissimilarity measure
				double const valDif{ delta2 - delta1 };

				// accumulate for variance of dissimilarity
				theSumSqDif += sq(valDif);
				++theCount;

				// track +/- balance for average of opposite cell pairs
				double const valSum{ delta2 + delta1 };
				if (valSum < 0.f)
				{
					++theNumNeg;
				}
				else
				{
					++theNumPos;
				}
			}

			//! Value halfway between theMin and theMax
			inline
			double
			middleValue
				() const
			{
				return (.5 * (theMax + theMin));
			}

			//! Range of values considered (max - min)
			inline
			double
			valueRange
				() const
			{
				return (theMax - theMin);
			}

			//! True if numPos and numNeg considered values more than minPosNeg
			inline
			bool
			hasPosNegBalance
				( std::size_t const & minPosNeg
				)
			{
				bool const enoughPos{ (minPosNeg < theNumPos) };
				bool const enoughNeg{ (minPosNeg < theNumNeg) };
				return (enoughPos && enoughNeg);
			}

			//! Standard deviation of values considered
			inline
			double
			sigmaValueDifs
				() const
			{
				double const valDifVar{ theSumSqDif / (double)theCount };
				double const valDifSig{ std::sqrt(valDifVar) };
				return valDifSig;
			}

			//! Pseudo-Probability that center value is between min/max
			inline
			double
			centerValueProb
				( double const & centerValue
				) const
			{
				double prob{ std::numeric_limits<double>::quiet_NaN() };
				if (isValid() && (theMin < theMax))
				{
					double const delta{ centerValue - middleValue() };
					double const sigma{ (1./4.) * valueRange() };
					double const arg{ delta / sigma };
					prob = std::exp(-sq(arg));
				}
				return prob;
			}

		}; // RingStats


		//! \brief Evaluate the metric at source image (row,col) location
		inline
		float
		operator()
			( std::size_t const & row
			, std::size_t const & col
			) const
		{
			float outVal{ std::numeric_limits<float>::quiet_NaN() };
			ras::Grid<float> const & srcGrid = *thePtSrc;

			// compare first and second half of ring
			// (ring pattern halves should repeat for half-turn symmetry)
			RingStats ringStats{};

			bool hitNull{ false };
			for (std::size_t nn{0u} ; nn < theHalfRingSize ; ++nn)
			{
				// check values at radially opposite portion of annulus
				std::size_t & ndx1 = nn;
				std::size_t ndx2{ ndx1 + theHalfRingSize };

				// access radially opposite ring source values
				ras::RelRC const & relRC1 = theRelRCs[ndx1];
				float const & srcVal1 = srcGrid(relRC1.srcRowCol(row, col));

				ras::RelRC const & relRC2 = theRelRCs[ndx2];
				float const & srcVal2 = srcGrid(relRC2.srcRowCol(row, col));

				if (pix::isValid(srcVal1) && pix::isValid(srcVal2))
				{
					double const delta1{ (double)(srcVal1 - theSrcMidValue) };
					double const delta2{ (double)(srcVal2 - theSrcMidValue) };
					ringStats.consider(delta1, delta2);
				}
				else
				{
					hitNull = true;
					break;
				}

			} // ring loop

			// perform filter analysis
			if (! hitNull)
			{
				// Balanced lo/hi count threshold qualification
				if (! ringStats.hasPosNegBalance(theMinPosNeg))
				{
					outVal = 0.f;
				}
				else // if (enoughPos && enoughNeg)
				{
					// Half-Turn Symmetry metric
					double const valDifSig{ ringStats.sigmaValueDifs() };
					double const valSigma{ theSrcFullRange / 8. };
					double const valDifRatio{ valDifSig / valSigma };
					double const valDifProb{ std::exp(-sq(valDifRatio)) };

					// High Contrast metric
					double const rngRing{ ringStats.valueRange() };

					// Center element has value near middle of range
				//	float const & midVal = srcGrid(row, col);
				//	double const midProb{ ringStats.centerValueProb(midVal) };

					// filter response value
					outVal = (float)(rngRing * valDifProb);
				//	outVal = (float)(rngRing * valDifProb * midProb);
				}
			}

			return outVal;
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
				oss << title << '\n';
			}
			if (thePtSrc)
			{
				oss
					<< "*thePtSrc: " << *thePtSrc
					<< '\n'
					<< "theSrcMidValue: " << theSrcMidValue
					<< '\n'
					<< "theSrcFullRange: " << theSrcFullRange
					<< '\n'
					<< "theHalfFilterSize: " << theHalfFilterSize
					<< '\n'
					<< "theHalfRingSize: " << theHalfRingSize
					<< '\n'
					<< "theMinPosNeg: " << theMinPosNeg
					<< '\n'
					<< "theRelRCs.size: " << theRelRCs.size()
					<< '\n'
					;
			}
			return oss.str();
		}

		//! Descriptive information about this instance.
		inline
		std::string
		infoStringContents
			( std::string const & title = {}
			) const
		{
			std::ostringstream oss;
			oss << infoString(title) << '\n';
			for (ras::RelRC const & relRC : theRelRCs)
			{
				oss << "... relRC: " << relRC << '\n';
			}
			return oss.str();
		}



	}; // SymRing


	//! \brief Result applying SymRing rotation symmetry filter to full grid
	inline
	ras::Grid<float>
	symRingGridFor
		( ras::Grid<float> const & srcGrid
			//!< Input intensity grid
		, SymRing const & symRing
			//!< Annular symmetry filter
		)
	{
		ras::Grid<float> symGrid(srcGrid.hwSize());
		std::fill(symGrid.begin(), symGrid.end(), pix::fNull);

		std::size_t const halfSize{ symRing.halfSize() };
		std::size_t const fullSize{ symRing.fullSize() };
		if ((fullSize < srcGrid.high()) && (fullSize < srcGrid.high()))
		{
			std::size_t const & rowBeg = halfSize;
			std::size_t const & colBeg = halfSize;
			std::size_t const rowEnd{ srcGrid.high() - halfSize };
			std::size_t const colEnd{ srcGrid.wide() - halfSize };
			for (std::size_t row{rowBeg} ; row < rowEnd ; ++row)
			{
				for (std::size_t col{colBeg} ; col < colEnd ; ++col)
				{
					symGrid(row, col) = symRing(row, col);
				}
			}
		}

		return symGrid;
	}

	//! \brief Result applying SymRing rotation symmetry filter to full grid
	inline
	ras::Grid<float>
	symRingGridFor
		( ras::Grid<float> const & srcGrid
			//!< Input intensity grid
		, std::size_t const & ringHalfSize
			//!< Annular filter radius. \ref SymRing constructor
		)
	{
		prb::Stats<float> const srcStats(srcGrid.cbegin(), srcGrid.cend());
		SymRing const symRing(&srcGrid, srcStats, ringHalfSize);
		return symRingGridFor(srcGrid, symRing);
	}

} // [ops]

} // [quadloco]

namespace
{

	//! Put instance to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::ops::SymRing const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

} // [anon/global]

