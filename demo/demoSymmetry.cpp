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
#include "opsPeakFinder2D.hpp"
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
	reportMax
		( ras::Grid<float> const & fGrid
		)
	{
		using Iter = ras::Grid<float>::const_iterator;

		std::pair<Iter, Iter> const itMinMax
			{ ops::grid::minmax_valid(fGrid.cbegin(), fGrid.cend()) };
		Iter const & itMax = itMinMax.second;

		ras::RowCol const rcMax{ fGrid.rasRowColFor(itMax) };
		std::cout << "@@@@ rcMax:"
			<< ' ' << rcMax
			<< ' ' << fGrid(rcMax)
			<< '\n';
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

	// report largest peak
	reportMax(saveGrid);

	// get all peaks
	std::vector<ras::PeakRCV> const peakRCVs
		{ ops::PeakFinder2D::sortedPeakRCVs(saveGrid) };
	// display several of the largest magnitude ones
	std::size_t const numShow{ std::min(10u, (unsigned)peakRCVs.size()) };
	for (std::size_t nn{0u} ; nn < numShow ; ++nn)
	{
		ras::PeakRCV const & peakRCV = peakRCVs[nn];
		std::cout << "==> peakRCV: " << peakRCV << '\n';
	}
	if (numShow < peakRCVs.size())
	{
		std::cout << "==> peakRCV: ...\n";
	}

	// save filtered result
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



