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


#include "QuadLoco/appkeyed.hpp"
#include "QuadLoco/io.hpp"
#include "QuadLoco/rasGrid.hpp"

#include <filesystem>
#include <iostream>
#include <map>
#include <string>


/*! \brief Identify quad target center points in loaded PGM image
 *
 * Load an image and a collection of row/col locations (in *.meapoint format)
 * For each of the loaded row/col locations, extract an image chip centered
 * on that location (ref hardcoded chip size). Each chip is fed into the
 * center localization algorithm which uses two steps:
 * \arg 1) annular symmetry filter for candidate peak finding; and
 * \arg 2) a half-turn SSD filter for refining the peak location.
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
			"  Program that determines refined center points for real image"
			"\n"
			"\nAn image (PGM format only) loaded as source data. Assuming"
			"\na corresponding <srcFile>.meapoint file exists, it's contents"
			"\nare loaded and each location is used to define the center of"
			"\nan image chip."
			"\n"
			"\nEach chip definition is used to extract a small sub grid"
			"\nfrom the source image. This grid is processed to determine"
			"\na best hit that represents and estimated quad center (or"
			"\na null value if none can be determined"
			"\n"
			"\nResults are reported to standard output."
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
	using app::keyed::QuadKey;

	// Tuning constants
	ras::SizeHW const hwChip{ 64u, 64u };
//	std::vector<std::size_t> const ringHalfSizes{ 5u, 3u };
	std::vector<std::size_t> const ringHalfSizes{ 9u, 7u };

	// find centers for all chip specs
	ras::Grid<std::uint8_t> const loadGrid{ io::readPGM(pgmPath) };

	// Load nominal center locations from corresponding .meapoint file
	std::map<QuadKey, ras::RowCol> const keyNominalRCs
		{ app::keyed::keyMeaPointRCs(meaPath) };

	// Define chip specs centered on locations of interest
	std::map<QuadKey, ras::ChipSpec> const keyChips
		{ app::keyed::keyChipSpecsFor
			(keyNominalRCs, hwChip, loadGrid.hwSize())
		};


	// Find and refine quad target center locatinos
	std::map<QuadKey, img::Hit> const keyCenterHits
		{ app::keyed::keyCenterHitsNearTo(keyChips, loadGrid, ringHalfSizes) };


	// report input nominal values
	std::cout << "\n(nominal)ChipCenters\n";
	for (std::map<QuadKey, ras::RowCol>::value_type
		const & keyNominalRC : keyNominalRCs)
	{
		std::cout << "  "
			<< ' ' << keyNominalRC.first
			<< ' ' << keyNominalRC.second
			<< '\n';
	}

	// report found refined centers
	std::cout << "\nSrcCenterHits\n";
	for (std::map<QuadKey, img::Hit>::value_type
		const & keyCenterHit : keyCenterHits)
	{
		std::cout << "  "
			<< ' ' << keyCenterHit.first
			<< ' ' << keyCenterHit.second
			<< '\n';
	}

	// Optionally save fit locations
	constexpr bool saveChips{ true };
	if (saveChips)
	{
		for (std::map<QuadKey, img::Hit>::value_type
			const & keyCenterHit : keyCenterHits)
		{
			QuadKey const & key = keyCenterHit.first;

			// generate local filename paths
			std::ostringstream txt;
			txt << "./fit" << key << ".pgm";
			std::filesystem::path const fitPathPgm(txt.str());
			std::filesystem::path fitPathMea{ fitPathPgm };
			fitPathMea.replace_extension("meapoint");

			std::map<QuadKey, ras::ChipSpec>::const_iterator
				const itChip{ keyChips.find(key) };
			if (keyChips.cend() != itChip)
			{
				ras::ChipSpec const & chipSpec = itChip->second;

				// crop grid from source and convert to float
				ras::Grid<float> const srcGrid
					{ ras::grid::subGridValuesFrom<float>(loadGrid, chipSpec) };

				// get center estimate info
				img::Hit const & chipHit = keyCenterHit.second;
				double const sigma{ chipHit.sigma() };
				double const covar{ sigma * sigma };
				img::Spot const chipSpot
					{ chipSpec.chipSpotForFullSpot(chipHit.location()) };

				// save image chip
				(void)io::writeStretchPGM(fitPathPgm, srcGrid);

				// save center measurements
				std::ofstream ofs(fitPathMea);
				ofs << key
					<< ' ' << chipSpot
					<< ' ' << covar
					<< ' ' << 0.
					<< ' ' << covar
					<< '\n';
			}
		}

	}

	// summarize process
	std::string const pad("  ");
	std::cout << '\n';
	std::cout << "application: "
		<< std::filesystem::canonical(appPath).native() << '\n';
	std::cout << pad << "    pgmPath: " << pgmPath.native() << '\n';
	std::cout << pad << "    meaPath: " << meaPath.native() << '\n';
	std::cout << pad << " numNominal: " << keyNominalRCs.size() << '\n';
	std::cout << pad << " numCenters: " << keyCenterHits.size() << '\n';
//	std::cout << pad << "savePath: " << savePath.native() << '\n';
	std::cout << '\n';


	return 0;
}


