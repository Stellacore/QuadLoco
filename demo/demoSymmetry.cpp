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

		inline
		std::size_t
		srcIndexFor
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

		inline
		ras::RowCol
		srcRowCol
			( std::size_t const & srcRow0
			, std::size_t const & srcCol0
			) const
		{
			return ras::RowCol
				{ srcIndexFor(srcRow0, theRelRow)
				, srcIndexFor(srcCol0, theRelCol)
				};
		}

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


	struct SymRing
	{
		ras::Grid<float> const * const & thePtSrc{ nullptr };
		float theMidValue{};

		int const theHalf{ 3 };
		std::vector<RelRC> theRelRCs{};

		inline
		explicit
		SymRing
			( ras::Grid<float> const * const & ptSrc
			, float const & midValue
			, std::size_t const & halfSize
			)
			: thePtSrc{ ptSrc }
			, theMidValue{ midValue }
			, theHalf{ static_cast<int>(halfSize + 1u) }
			, theRelRCs{}
		{
			std::size_t const numPerim{ 8u * halfSize }; // exceeding 2*pi*r^2
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

std::ostringstream msg;

for (RelRC const & relRC : theRelRCs)
{
	using engabra::g3::io::fixed;
	msg
		<< "  rc: " << fixed(relRC.theRelRow) << ' ' << fixed(relRC.theRelCol)
		<< '\n';
}

std::cout << msg.str() << '\n';

std::ofstream ofs("foo");
ofs << msg.str() << std::endl;

/*
exit(8);
*/

		}

		inline
		std::size_t
		halfSize
			() const
		{
			return static_cast<std::size_t>(theHalf);
		}

		inline
		std::size_t
		fullSize
			() const
		{
			return (2u*halfSize() + 1u);
		}

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
			static std::vector<float> ringVals{};
			if (ringVals.size() != theRelRCs.size())
			{
				ringVals.resize(theRelRCs.size());
			}


bool show{ (36u == row) && (69u == col) };
show = false;
if (show)
{
	std::cout << "row: " << row << "  col" << col << '\n';
}

			// annulus indices in monotonic angular sequence
			std::size_t const fullRingSize{ theRelRCs.size() };
			std::size_t const halfRingSize{ fullRingSize / 2u };

			// Extract ring values from source grid
			bool hitNull{ false };
			for (std::size_t nn{0u} ; nn < fullRingSize ; ++nn)
			{
				RelRC const & relRC = theRelRCs[nn];
				float const & srcVal = srcGrid(relRC.srcRowCol(row, col));
if (show)
{
	using engabra::g3::io::fixed;
	ras::RowCol const srcRC{ relRC.srcRowCol(row, col) };
	std::cout
		<< "  relRC:"
			<< ' ' << std::setw(3u) << relRC.theRelRow
			<< ' ' << std::setw(3u) << relRC.theRelCol
		<< "  srcRC:"
			<< ' ' << std::setw(3u) << srcRC.row()
			<< ' ' << std::setw(3u) << srcRC.col()
		<< "  srcVal: " << (srcVal - theMidValue)
		<< '\n';
}
				if (pix::isValid(srcVal))
				{
					float const delta{ srcVal - theMidValue };
					ringVals[nn] = delta;
				}
				else
				{
					hitNull = true;
				}
			}

			if (! hitNull)
			{
				// compare first and second half of ring
				// Ring pattern should repeat for half-turn symmetry
				// Note: logic could be put inside first loop above
				float sumSqDif{ 0.f };
				float min{ ringVals[0] };
				float max{ ringVals[0] };
				for (std::size_t nn{0u} ; nn < halfRingSize ; ++nn)
				{
					std::size_t & ndx1 = nn;
					std::size_t ndx2{ ndx1 + halfRingSize };

					float const & v1 = ringVals[ndx1];
					float const & v2 = ringVals[ndx2];

					min = std::min(v1, min);
					min = std::min(v2, min);
					max = std::max(v1, max);
					max = std::max(v2, max);

					float const dif{ v2 - v1 };

					sumSqDif += dif * dif;
				}

				float difVar{ sumSqDif / static_cast<float>(halfRingSize) };
				float difSig{ std::sqrtf(difVar) };

				constexpr float const sigma{ 256.f / 8.f };
				float const arg{ difSig / sigma };
				float const rng{ max - min };

using engabra::g3::io::fixed;
std::cout
	<< "  rng: " << fixed(rng, 4u, 1u)
	<< "  difSig: " << fixed(difSig, 4u, 1u)
	<< "  sigma: " << fixed(sigma, 4u, 1u)
	<< "  arg: " << fixed(arg)
	<< '\n';
/*
*/

				outVal = sumSqDif;
				outVal = 1.f/sumSqDif;
				outVal = difSig;
				outVal = arg;

				outVal = rng;
				outVal = rng * std::exp(-arg*arg);
			}

if (show)
{
	std::ostringstream msg;
	msg << "ringVals:\n";
	for (std::size_t nn{0u} ; nn < fullRingSize ; ++nn)
	{
		msg << ' ' << engabra::g3::io::fixed(ringVals[nn]);
	}
	std::cout << msg.str() << '\n';
}
/*
*/

			return outVal;
		}

	}; // SymRing


	inline
	ras::Grid<float>
	ringSymmetryFilter
		( ras::Grid<float> const & srcGrid
		, prb::Stats<float> const & srcStats
		, std::size_t const & ringHalfSize = 5u
		)
	{
		ras::Grid<float> symGrid(srcGrid.hwSize());
		std::fill(symGrid.begin(), symGrid.end(), pix::fNull);

		float const midValue{ .5f * (srcStats.max() + srcStats.min()) };
		SymRing const symRing(&srcGrid, midValue, ringHalfSize);

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

	prb::Stats<float> const srcStats(srcGrid.cbegin(), srcGrid.cend());
	std::cout << "\nsrcStats: " << srcStats << '\n';

/*
	ras::Grid<img::Grad> const gradGrid{ ops::grid::gradientGridBy8x(srcGrid) };
	ras::Grid<float> const magGrid{ sig::util::magnitudeGridFor(gradGrid) };
	foo(magGrid);

	constexpr std::size_t upFactor{ 1u };
	ras::Grid<float> saveGrid{ sig::util::toLarger(srcGrid, upFactor) };
*/
	ras::Grid<float> saveGrid{ ringSymmetryFilter(srcGrid, srcStats) };
	bool const okaySave{ io::writeStretchPGM(savePath, saveGrid) };

	std::cout << '\n';
	std::cout << "load : " << loadPath << ' ' << loadGrid << '\n';
	std::cout << "save : " << savePath << ' ' << saveGrid << '\n';
	std::cout << "okaySave: " << okaySave << '\n';
	std::cout << '\n';

	return 0;
}



