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


/*! \file
\brief Main application program to experiment with signal pattern symmetry
*/


#include "cast.hpp"
#include "io.hpp"
#include "opsFence.hpp"
#include "opsgrid.hpp"
#include "prbStats.hpp"
#include "rasGrid.hpp"
#include "sigutil.hpp"

#include <Engabra>

#include <algorithm>
#include <iostream>
#include <limits>
#include <numbers>
#include <vector>


namespace quadloco
{
	template <typename Type>
	inline
	Type
	sq
		( Type const & val
		)
	{
		return (val*val);
	}


	// Relative row/col position (wrt 'active' filter center pixel)
	struct RelRC
	{
		// offsets relative to filter center
		int theRelRow{};
		int theRelCol{};

		//! Compute sum of indices with casting/test to avoid negative offsets
		inline
		std::size_t
		unsignedIndexFor
			( std::size_t const & srcCenterNdx
			, int const & ndxDelta
			) const
		{
			int const iNdx{ static_cast<int>(srcCenterNdx) + ndxDelta };
			if (iNdx < 0)
			{
				std::cerr << "Failure of iNdx positive check\n";
				std::cerr << "  srcCenterNdx: " << srcCenterNdx << '\n';
				std::cerr << "      ndxDelta: " << ndxDelta << '\n';
				exit(1);
			}
			return static_cast<std::size_t>(iNdx);
		}

		//! Absolute source grid (row,col) given filter center (row0,col0)
		inline
		ras::RowCol
		srcRowCol
			( std::size_t const & srcRow0
			, std::size_t const & srcCol0
			) const
		{
			return ras::RowCol
				{ unsignedIndexFor(srcRow0, theRelRow)
				, unsignedIndexFor(srcCol0, theRelCol)
				};
		}

		//! True if this and other instances have identical relative indices.
		inline
		bool
		operator==
			( RelRC const & other
			) const
		{
			return
				(  (theRelRow == other.theRelRow)
				&& (theRelCol == other.theRelCol)
				);
		}
	};


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
		ras::Grid<float> const * const & thePtSrc{ nullptr };

		float theSrcMidValue{};
		float theSrcFullRange{};

		int const theHalf{};
		std::vector<RelRC> theRelRCs{};

		static constexpr std::size_t theMinPosNeg{ 1u };

		inline
		explicit
		SymRing
			( ras::Grid<float> const * const & ptSrc
				//!< Access source data grid (e.g. raw or smoothed image, etc)
			, prb::Stats<float> const & srcStats
				//!< Statistics on source data grid
			, std::size_t const & halfSize
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
			: thePtSrc{ ptSrc }
			, theSrcMidValue{ .5f * (srcStats.max() + srcStats.min()) }
			, theSrcFullRange{ srcStats.range() }
			, theHalf{ static_cast<int>(halfSize + 1u) }
			, theRelRCs{}
		{
			// perimeter shold be 2*pi*r < 7*r,
			// but allow for duplicates during initial compuation
			std::size_t const numPerim{ 2u * (7u * theHalf) };
			theRelRCs.reserve(numPerim);

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
				RelRC const relRC
					{ static_cast<int>(std::floor(.5 + spot[0]))
					, static_cast<int>(std::floor(.5 + spot[1]))
					};
				if (theRelRCs.empty())
				{
					theRelRCs.emplace_back(relRC);
				}
				else
				if (! (relRC == theRelRCs.front()))
				{
					theRelRCs.emplace_back(relRC);
				}
			}

			// fill second quadrant with reverse symmetry
			using Iter = std::vector<RelRC>::const_reverse_iterator;
			Iter const endQtr{ theRelRCs.crend() };
			for (Iter iter{theRelRCs.crbegin()} ; endQtr != iter ; ++iter)
			{
				RelRC const & relRC = *iter;
				RelRC const negRC{ -relRC.theRelRow,  relRC.theRelCol };
				theRelRCs.emplace_back(negRC);
			}

			// fill second half with half-turn symmetry
			std::size_t const halfEnd{ theRelRCs.size() - 1u };
			for (std::size_t nn{0u} ; nn < halfEnd ; ++nn)
			{
				RelRC const & relRC = theRelRCs[nn];
				RelRC const negRC{ -relRC.theRelRow, -relRC.theRelCol };
				theRelRCs.emplace_back(negRC);
			}

			// remove duplicate entries
			std::vector<RelRC>::iterator const newEnd
				{ std::unique(theRelRCs.begin(), theRelRCs.end()) };
			theRelRCs.resize(std::distance(theRelRCs.begin(), newEnd));

			/*
			std::cout << "theRelRCs.size(): " << theRelRCs.size() << '\n';
			for (RelRC const & relRC : theRelRCs)
			{
				std::cout << "relRC:"
					<< ' ' << relRC.theRelRow
					<< ' ' << relRC.theRelCol
					<< '\n';
			}
			*/

		}

		//! Nominal "radial" size of annulus
		inline
		std::size_t
		halfSize
			() const
		{
			return static_cast<std::size_t>(theHalf);
		}

		//! Nominal "diagonal" size of annulus
		inline
		std::size_t
		fullSize
			() const
		{
			return (2u*halfSize() + 1u);
		}

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

			// allocate space for filter ring values
			static std::vector<float> ringDeltas{};
			if (ringDeltas.size() != theRelRCs.size())
			{
				ringDeltas.resize(theRelRCs.size());
			}

			// annulus indices in monotonic angular sequence
			std::size_t const fullRingSize{ theRelRCs.size() };
			std::size_t const halfRingSize{ fullRingSize / 2u };

			// extract ring values from source grid
			bool hitNull{ false };
			for (std::size_t nn{0u} ; nn < fullRingSize ; ++nn)
			{
				RelRC const & relRC = theRelRCs[nn];
				float const & srcVal = srcGrid(relRC.srcRowCol(row, col));
				if (pix::isValid(srcVal))
				{
					// use deltas relative to source grid overall middle value
					float const delta{ srcVal - theSrcMidValue };
					ringDeltas[nn] = delta;
				}
				else
				{
					hitNull = true;
				}
			}

			// perform filter analysis
			if (! hitNull)
			{
				// compare first and second half of ring
				// Ring pattern should repeat for half-turn symmetry
				// Note: logic could be put inside first loop above
				float sumSqDif{ 0.f };
				// track contrast limits over annulus
				float min{ ringDeltas[0] };
				float max{ ringDeltas[0] };
				// track balance of +/- values in the annulus
				float numPos{ 0.f };
				float numNeg{ 0.f };
				for (std::size_t nn{0u} ; nn < halfRingSize ; ++nn)
				{
					// check values at radially opposite portion of annulus
					std::size_t & ndx1 = nn;
					std::size_t ndx2{ ndx1 + halfRingSize };

					float const & delta1 = ringDeltas[ndx1];
					float const & delta2 = ringDeltas[ndx2];

					// track annulus min/max
					min = std::min(delta1, min);
					min = std::min(delta2, min);
					max = std::max(delta1, max);
					max = std::max(delta2, max);

					// compute dissimilarity measure
					float const valDif{ delta2 - delta1 };

					// accumulate for variance of dissimilarity
					sumSqDif += sq(valDif);

					// track +/- balance for average of opposite cell pairs
					float const valSum{ delta2 + delta1 };
					if (valSum < 0.f)
					{
						++numNeg;
					}
					else
					{
						++numPos;
					}
				}

				// Balanced lo/hi count threshold qualification
				bool const enoughPos{ (theMinPosNeg < numPos) };
				bool const enoughNeg{ (theMinPosNeg < numNeg) };
				if (! (enoughPos && enoughNeg))
				{
					outVal = 0.f;
				}
				else // if (enoughPos && enoughNeg)
				{
					// Half-Turn Symmetry metric
					float valDifVar
						{ sumSqDif / static_cast<float>(halfRingSize) };
					float valDifSig{ std::sqrtf(valDifVar) };
					float const valSigma{ theSrcFullRange / 8.f };
					float const valArg{ valDifSig / valSigma };

					// High Contrast metric
					float const rngRing{ max - min };

					// filter response value
					outVal = rngRing * std::exp(-sq(valArg));
				}
			}

			return outVal;
		}

	}; // SymRing


	//! \brief Result of applying an annular ring rotation symmetry filter.
	inline
	ras::Grid<float>
	ringSymmetryFilter
		( ras::Grid<float> const & srcGrid
			//!< Input intensity grid
		, std::size_t const & ringHalfSize
			//!< Annular filter radius. \ref SymRing constructor
		)
	{
		ras::Grid<float> symGrid(srcGrid.hwSize());
		std::fill(symGrid.begin(), symGrid.end(), pix::fNull);

		prb::Stats<float> const srcStats(srcGrid.cbegin(), srcGrid.cend());
		std::cout << "\nsrcStats: " << srcStats << '\n';

		SymRing const symRing(&srcGrid, srcStats, ringHalfSize);

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

} // [quadloco]


/*! \brief Experiment with point of symmetry metrics
*/
int
main
	( int argc
	, char * argv[]
	)
{
	if (! (2 < argc))
	{
		std::cerr << '\n';
		std::cerr << "Run signal symmetry filter on input PGM file\n";
		std::cerr << '\n';
		std::cerr << "Usage: <progname> <InputPGM> <OutputPGM>\n";
		std::cerr << '\n';
		return 1;
	}
	std::size_t ndx{ 1u };
	std::filesystem::path const loadPath(argv[ndx++]);
	std::filesystem::path const savePath(argv[ndx++]);

	using namespace quadloco;

	// load image
	ras::Grid<std::uint8_t> const loadGrid{ io::readPGM(loadPath) };
	ras::Grid<float> const srcGrid{ sig::util::toFloat(loadGrid, 0u) };

	/*
	prb::Stats<float> const srcStats(srcGrid.cbegin(), srcGrid.cend());
	std::cout << "\nsrcStats: " << srcStats << '\n';
	*/

	constexpr std::size_t ringHalfSize{ 5u };
	ras::Grid<float> saveGrid{ ringSymmetryFilter(srcGrid, ringHalfSize) };
	bool const okaySave{ io::writeStretchPGM(savePath, saveGrid) };

//(void)io::writeStretchPGM(std::filesystem::path("0src.pgm"), srcGrid);

	std::cout << '\n';
	std::cout << "load : " << loadPath << ' ' << loadGrid << '\n';
	std::cout << "save : " << savePath << ' ' << saveGrid << '\n';
	std::cout << "okaySave: " << okaySave << '\n';
	std::cout << '\n';

	return 0;
}



