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
	sig::QuadTarget
	bestQuadTargetFor
		( ras::Grid<float> const & srcGrid
		)
	{
		sig::QuadTarget sigQuad;
		ras::Grid<img::Grad> const gradGrid
			{ ops::grid::gradientGridFor(srcGrid) };
		sig::EdgeEval const edgeEval(gradGrid);
		std::vector<sig::QuadWgt> const sigQuadWgts
			{ edgeEval.sigQuadWeights(gradGrid.hwSize()) };
		if (! sigQuadWgts.empty())
		{
			sigQuad = sigQuadWgts.front().item();
		}

		(void)io::writeStretchPGM("grid00Input.pgm", srcGrid);

		ras::Grid<float> const gridEdgeDom
			{ edgeEval.infoGridDominantEdgelMag(gradGrid) };
		(void)io::writeStretchPGM("grid01EdgeDom.pgm", gridEdgeDom);

		ras::Grid<float> const gridRadLike
			{ edgeEval.infoGridLikelyRadial(gradGrid) };
		(void)io::writeStretchPGM("grid02RadLike.pgm", gridRadLike);

std::cout << "gradGrid: " << gradGrid << '\n';

// EdgeInfo-mag
ras::Grid<float> const edgeInfoWeightGrid
	{ sig::util::edgeInfoWeightGrid
		(gradGrid.hwSize(), edgeEval.edgeInfos())
	};
(void)io::writeStretchPGM("edgeInfoWgt.pgm", edgeInfoWeightGrid);

		return sigQuad;
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
	ras::Grid<float> const useGrid{ sig::util::toFloat(srcGrid) };

	// find center
	sig::QuadTarget const sigQuad{ bestQuadTargetFor(useGrid) };
	img::Spot const useCenter{ sigQuad.centerSpot() };

	// upsample image and draw center in enlarged grid
	constexpr std::size_t upFactor{ 8u };
	ras::Grid<float> outGrid{ sig::util::toLarger(useGrid, upFactor) };
	img::Spot const outCenter{ (double)upFactor * useCenter };
	sig::util::drawSpot(&outGrid, outCenter);

	// save (enlarged) image with center drawn
	bool const okaySave{ io::writeStretchPGM(outPath, outGrid) };

	std::cout << '\n';
	std::cout << "sigQuad: " << sigQuad << '\n';
	std::cout << '\n';
	std::cout << "load : " << srcPath << ' ' << srcGrid << '\n';
	std::cout << "save : " << outPath << ' ' << outGrid << '\n';
	std::cout << "okaySave: " << okaySave << '\n';
	std::cout << '\n';

	return 0;
}


