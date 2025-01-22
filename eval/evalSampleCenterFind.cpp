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
\brief Identify quad center points from arbitrary PGM image
*/


#include "appCenters.hpp"
#include "io.hpp"
#include "opsCenterRefinerSSD.hpp"
#include "rasChipSpec.hpp"
#include "rasgrid.hpp"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <string>


namespace quadloco
{

namespace app
{

	//! center feature hit via multiSymRingPeaks and CenterRefinerSSD.
	inline
	img::Hit
	refinedCenterHitFrom
		( ras::Grid<float> const & srcGrid
		, std::vector<std::size_t> const & halfRingSizes = { 5u, 3u  }
		)
	{
		img::Hit centerHit;

		// find nominal peaks
		std::vector<ras::PeakRCV> peakRCVs
			{ app::multiSymRingPeaks(srcGrid, halfRingSizes) };

		if (! peakRCVs.empty())
		{
			// refine peak with max value
			std::vector<ras::PeakRCV>::const_iterator const itMax
				{ std::max_element(peakRCVs.cbegin(), peakRCVs.cend()) };
			if (peakRCVs.cend() != itMax)
			{
				ras::PeakRCV const & peakRCV = *itMax;
				ops::CenterRefinerSSD const refiner(&srcGrid);
				centerHit = refiner.fitHitNear(peakRCV.theRowCol);
			}
		}
		return centerHit;
	}

	//! Creat ChipSpec instances for each source center location
	inline
	std::vector<ras::ChipSpec>
	chipSpecsFor
		( std::vector<ras::RowCol> const & rcChipCenters
		, ras::SizeHW const & hwChip
		, ras::SizeHW const & hwFull
		)
	{
		// Generate crop areas centered on rcChipCenters and inside source grid
		std::vector<ras::ChipSpec> chipSpecs{};
		chipSpecs.reserve(rcChipCenters.size());

		std::size_t const chipHalfHigh{ hwChip.high() / 2u };
		std::size_t const chipHalfWide{ hwChip.wide() / 2u };
		for (ras::RowCol const & rcChipCenter : rcChipCenters)
		{
			if ( (chipHalfHigh < rcChipCenter.row())
			  && (chipHalfWide < rcChipCenter.col())
			   )
			{
				ras::RowCol const chipOrig
					{ (rcChipCenter.row() - chipHalfHigh)
					, (rcChipCenter.col() - chipHalfWide)
					};
				ras::ChipSpec const chipSpec{ chipOrig, hwChip };
				if (chipSpec.fitsInto(hwFull))
				{
					chipSpecs.emplace_back(chipSpec);
				}
			}
		}

		return chipSpecs;
	}

	//! Load img::Spot locations from *.meaPnt file (row,col only)
	inline
	std::vector<ras::RowCol>
	meaPointRCs
		( std::filesystem::path const & path
		)
	{
		std::vector<ras::RowCol> rcs;
		rcs.reserve(128u);
		std::ifstream ifs(path);
		std::string line;
		std::string id;
		double row;
		double col;
		while (ifs.good() && (! ifs.eof()))
		{
			std::getline(ifs, line);
			std::istringstream iss(line);
			iss >> id >> row >> col;
			if (! iss.bad())
			{
				img::Spot const spot{ row, col };
				ras::RowCol const rc{ cast::rasRowCol(spot) };
				rcs.emplace_back(rc);
			}
		}
		return rcs;
	}

} // [app]

} // [quadloco]


/*! \brief Identify quad target center points in loaded PGM image
*/
int
main
	( int argc
	, char * argv[]
	)
{
	if (! (1 < argc))
	{
		std::cerr << '\n';
		std::cerr << "Run quad center finding on loaded image\n";
		std::cerr << '\n';
		std::cerr << "Usage: <progname> <srcFile.pgm\n";
		std::cerr << '\n';
		std::cerr << 
			"  Program ...." //TODO
			"\n\n"
			;
		return 1;
	}

	// Setup paths

	std::filesystem::path const appPath(argv[0]);
	std::filesystem::path const pgmPath(argv[1]);
	std::filesystem::path meaPath{ pgmPath };
	meaPath.replace_extension("meapoint");

	// load source grid and find centers

	using namespace quadloco;

	// find centers for all chip specs
	ras::Grid<std::uint8_t> const loadGrid{ io::readPGM(pgmPath) };

	// Load nominal center locations from corresponding .meapoint file
	ras::SizeHW const hwChip{ 64u, 64u };
	std::vector<ras::RowCol> const rcChipCenters{ app::meaPointRCs(meaPath) };

	// Define chip specs centered on locations of interest
	std::vector<ras::ChipSpec> const chipSpecs
		{ app::chipSpecsFor(rcChipCenters, hwChip, loadGrid.hwSize()) };

	// Extract refined center locations for each chip
	std::vector<img::Hit> centerHits{};
	centerHits.reserve(chipSpecs.size());
	for (ras::ChipSpec const & chipSpec : chipSpecs)
	{
		// crop grid from source and convert to float
		ras::Grid<float> const srcGrid
			{ ras::grid::subGridValuesFrom<float>(loadGrid, chipSpec) };

		img::Hit const chipHit{ app::refinedCenterHitFrom(srcGrid) };
		// adjust hit to reflect source grid coordinates
		img::Hit const centerHit
			{ chipSpec.fullSpotForChipSpot(chipHit.location())
			, chipHit.value()
			, chipHit.sigma()
			};
		centerHits.emplace_back(centerHit);

	}

	std::cout << "\n(nominal)ChipCenters\n";
	for (ras::RowCol const & rcChipCenter : rcChipCenters)
	{
		std::cout << "  " << rcChipCenter << '\n';
	}

	std::cout << "\nSrcCenterHits\n";
	for (img::Hit const & centerHit : centerHits)
	{
		std::cout << "  " << centerHit << '\n';
	}


	// report results

	std::string const pad("  ");
	std::cout << '\n';
	std::cout << "application: "
		<< std::filesystem::canonical(appPath).native() << '\n';
	std::cout << pad << "    pgmPath: " << pgmPath.native() << '\n';
	std::cout << pad << "    meaPath: " << meaPath.native() << '\n';
	std::cout << pad << "   numChips: " << chipSpecs.size() << '\n';
	std::cout << pad << " numCenters: " << centerHits.size() << '\n';
//	std::cout << pad << "savePath: " << savePath.native() << '\n';
	std::cout << '\n';


	return 0;
}


