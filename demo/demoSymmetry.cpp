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
#include "opsSymRing.hpp"
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
	//! Print the row/col location of the largest value in fGrid
	inline
	void
	peakSearch
		( ras::Grid<float> const & fGrid
		)
	{
		std::vector<ras::RowCol> peakRCs;
		peakRCs.reserve(4u*1024u);

		std::size_t const high{ fGrid.high() };
		std::size_t const wide{ fGrid.wide() };
		std::size_t const lastRow{ high - 1u };
		std::size_t const lastCol{ wide - 1u };
		for (std::size_t currRow{1u} ; currRow < lastRow ; ++currRow)
		{
			std::size_t const prevRow{ currRow - 1u };
			std::size_t const nextRow{ currRow + 1u };
			for (std::size_t currCol{1u} ; currCol < lastCol ; ++currCol)
			{
				std::size_t const prevCol{ currCol - 1u };
				std::size_t const nextCol{ currCol + 1u };

				float const & TL = fGrid(prevRow, prevCol);
				float const & TM = fGrid(prevRow, currCol);
				float const & TR = fGrid(prevRow, nextCol);

				float const & ML = fGrid(currRow, prevCol);
				float const & MM = fGrid(currRow, currCol);
				float const & MR = fGrid(currRow, nextCol);

				float const & BL = fGrid(nextRow, prevCol);
				float const & BM = fGrid(nextRow, currCol);
				float const & BR = fGrid(nextRow, nextCol);

float const minValue{ std::numeric_limits<float>::epsilon() };
				if (minValue < MM)
				{

					if (  pix::isValid(TL)
					   && pix::isValid(TM)
					   && pix::isValid(TR)
					   && pix::isValid(ML)
					   && pix::isValid(MM)
					   && pix::isValid(MR)
					   && pix::isValid(BL)
					   && pix::isValid(BM)
					   && pix::isValid(BR)
					   )
					{
						bool const isPeak
							{  (! (MM < TL)) && (! (MM < TM)) && (! (MM < TR))
							&& (! (MM < ML))                  && (! (MM < MR))
							&& (! (MM < BL)) && (! (MM < BM)) && (! (MM < BR))
							};
						if (isPeak)
						{
							peakRCs.emplace_back
								(ras::RowCol{ currRow, currCol });

						} // isPeak

					} // all valid

				} // minValue

			}
		}

// TODO - add to a heap then can inspect and test in priority order
		for (ras::RowCol const & peakRC : peakRCs)
		{
			using engabra::g3::io::fixed;
			std::cout
				<< "==> peakRC: " << peakRC
				<< ' ' << fixed(fGrid(peakRC), 4u, 3u)
				<< '\n';
		}
	}

	//! Print the row/col location of the largest value in fGrid
	inline
	void
	reportMax
		( ras::Grid<float> const & fGrid
		)
	{
		using Iter = ras::Grid<float>::const_iterator;

		std::pair<Iter, Iter> const itMinMax
			{ ops::grid::minmax_valid(fGrid.cbegin(), fGrid.cend()) };
		Iter const & itMax = itMinMax.second;

		ras::RowCol const rcMax{ fGrid.rasRowColFor(itMax) };
		std::cout << "@@@@ rcMax: " << rcMax << '\n';
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
	ras::Grid<float> saveGrid{ ops::symRingGridFor(srcGrid, ringHalfSize) };

reportMax(saveGrid);
	peakSearch(saveGrid);

	bool const okaySave{ io::writeStretchPGM(savePath, saveGrid) };

//(void)io::writeStretchPGM(std::filesystem::path("0src.pgm"), srcGrid);

	std::cout << '\n';
	std::cout << "load : " << loadPath << ' ' << loadGrid << '\n';
	std::cout << "save : " << savePath << ' ' << saveGrid << '\n';
	std::cout << "okaySave: " << okaySave << '\n';
//std::cout << "\n*NOTE* - saving copy of input to 0src.pgm\n";
	std::cout << '\n';

	return 0;
}



