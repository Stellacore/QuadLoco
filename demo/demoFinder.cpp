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
\brief Contains example application demoFinder
*/


#include "cast.hpp"
#include "imgArea.hpp"
#include "io.hpp"
#include "opsgrid.hpp"
#include "rasGrid.hpp"
#include "sigEdgeEval.hpp"
#include "sigutil.hpp"

#include <algorithm>
#include <filesystem>
#include <iostream>


namespace quadloco
{
	//! Return "best" quad (one with highest detection weight)
	inline
	img::QuadTarget
	bestQuadTargetFor
		( ras::Grid<float> const & srcGrid
		)
	{
		std::filesystem::path srcPgmPath("df1_srcGrid.pgm");
		std::filesystem::path srcDatPath("df1_srcGrid.dat");
		std::filesystem::path gmagPgmPath("df2_gmagGrid.pgm");
		std::filesystem::path gmagDatPath("df2_gmagGrid.dat");
		std::filesystem::path gangPgmPath("df3_gangGrid.pgm");
		std::filesystem::path gangDatPath("df3_gangGrid.dat");
		std::filesystem::path corrPgmPath("df4_corrGrid.pgm");
		std::filesystem::path corrDatPath("df4_corrGrid.dat");
		std::filesystem::path radPgmPath("df5_radGrid.pgm");
		std::filesystem::path radDatPath("df5_radGrid.dat");


		constexpr bool saveDiagnostics{ true };

		//
		// Start with source image
		//
		img::QuadTarget imgQuad;
		if (saveDiagnostics)
		{
			bool const okPgm
				{ io::writeStretchPGM(srcPgmPath, srcGrid) };
			bool const okDat
				{ io::writeAsciiFile(srcDatPath, srcGrid, "%9.3f", -1.f) };
			std::cout << "@@@ writing: " << srcPgmPath << okPgm << '\n';
			std::cout << "@@@ writing: " << srcDatPath << okDat << '\n';
		}

		//
		// Gradient grid (in place gradients)
		//
		ras::Grid<img::Grad> const gradGrid
			{ ops::grid::gradientGridBy8x(srcGrid) };
		if (saveDiagnostics)
		{
			{
			ras::Grid<float> const gmagGrid
				{ sig::util::magnitudeGridFor(gradGrid) };
			bool const okPgm
				{ io::writeStretchPGM(gmagPgmPath, gmagGrid) };
			bool const okDat
				{ io::writeAsciiFile(gmagDatPath, gmagGrid, "%9.3f", -1.f) };
			std::cout << "@@@ writing: " << gmagPgmPath << okPgm << '\n';
			std::cout << "@@@ writing: " << gmagDatPath << okDat << '\n';
			}

			{
			ras::Grid<float> const gangGrid
				{ sig::util::angleGridFor(gradGrid) };
			bool const okPgm
				{ io::writeStretchPGM(gangPgmPath, gangGrid) };
			bool const okDat
				{ io::writeAsciiFile(gangDatPath, gangGrid, "%9.6f", -1.f) };
			std::cout << "@@@ writing: " << gangPgmPath << okPgm << '\n';
			std::cout << "@@@ writing: " << gangDatPath << okDat << '\n';
			}

		}

		//
		// Edgel Collections
		//
		sig::EdgeEval const edgeEval(gradGrid);
		std::vector<sig::QuadWgt> const imgQuadWgts
			{ edgeEval.imgQuadWeights(gradGrid.hwSize()) };
for (sig::QuadWgt const & imgQuadWgt : imgQuadWgts)
{
	std::cout << "imgQuadWgt:\n" << imgQuadWgt << '\n';
}
		if (! imgQuadWgts.empty())
		{
			imgQuad = imgQuadWgts.front().item();
		}

		if (saveDiagnostics)
		{
			{
			ras::Grid<float> const corrGrid
				{ edgeEval.infoGridDominantEdgelMag(gradGrid) };
			bool const okPgm
				{ io::writeStretchPGM(corrPgmPath, corrGrid) };
			bool const okDat
				{ io::writeAsciiFile(corrDatPath, corrGrid, "%9.3f", -1.f) };
			std::cout << "@@@ writing: " << corrPgmPath << okPgm << '\n';
			std::cout << "@@@ writing: " << corrDatPath << okDat << '\n';
			}

			{
			ras::Grid<float> const radGrid
				{ edgeEval.infoGridLikelyRadial(gradGrid) };
			bool const okPgm
				{ io::writeStretchPGM(radPgmPath, radGrid) };
			bool const okDat
				{ io::writeAsciiFile(radDatPath, radGrid, "%9.3f", -1.f) };
			std::cout << "@@@ writing: " << radPgmPath << okPgm << '\n';
			std::cout << "@@@ writing: " << radDatPath << okDat << '\n';
			}

		}

		return imgQuad;
	}

} // [quadloco]


/*! \brief Application: load PGM, find quad center, and save with center spot.
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
		std::cerr << "Run quad center detection on input PGM file\n";
		std::cerr << '\n';
		std::cerr << "Usage: <progname> <InputPGM> <OutputPGM>\n";
		std::cerr << '\n';
		return 1;
	}
	std::size_t ndx{ 1u };
	std::filesystem::path const srcPath(argv[ndx++]);
	std::filesystem::path const outPath(argv[ndx++]);

	using namespace quadloco;

	// load image
	ras::Grid<std::uint8_t> const srcGrid{ io::readPGM(srcPath) };
	ras::Grid<float> const useGrid{ sig::util::toFloat(srcGrid, 0) };

	// smooth source signal
	ras::Grid<float> const softGrid
		{ ops::grid::smoothGridFor<float>(useGrid, 5u, 2.5) };

	// find center
	img::QuadTarget const imgQuad{ bestQuadTargetFor(softGrid) };
	img::Spot const useCenter{ imgQuad.centerSpot() };

	// upsample image and draw center in enlarged grid
	constexpr std::size_t upFactor{ 8u };
	ras::Grid<float> outGrid{ sig::util::toLarger(useGrid, upFactor) };
	img::Spot const outCenter{ (double)upFactor * useCenter };
	sig::util::drawSpot(&outGrid, outCenter);

	// save (enlarged) image with center drawn
	bool const okaySave{ io::writeStretchPGM(outPath, outGrid) };

	std::cout << '\n';
	std::cout << "imgQuad: " << imgQuad << '\n';
	std::cout << '\n';
	std::cout << "load : " << srcPath << ' ' << srcGrid << '\n';
	std::cout << "save : " << outPath << ' ' << outGrid << '\n';
	std::cout << "okaySave: " << okaySave << '\n';
	std::cout << '\n';

	return 0;
}


