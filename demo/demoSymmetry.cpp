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

	struct PairStats
	{
		float theAve;
		float theVar;

		inline
		explicit
		PairStats
			( float const & val1
			, float const & val2
			)
			: theAve{ .5f * (val1 + val2) }
			, theVar{ sq(val1-theAve) + sq(val2-theAve) }
		{ }

	}; // PairStats


	struct VecStats
	{
		float const theAve;
		float const theVar;

		template <typename FwdIter>
		inline
		static
		float
		averageOf
			( FwdIter const & beg
			, FwdIter const & end
			)
		{
			float ave{ std::numeric_limits<float>::quiet_NaN() };
			float sum{ 0.f };
			float count{ 0.f };
			for (FwdIter iter{beg} ; end != iter ; ++iter)
			{
				sum += *iter;
				count += 1.f;
			}
			if (0.f < count)
			{
				ave = sum / count;
			}
			return ave;
		}

		template <typename FwdIter>
		inline
		static
		float
		varianceOf
			( FwdIter const & beg
			, FwdIter const & end
			, float const & ave
			)
		{
			float var{ std::numeric_limits<float>::quiet_NaN() };
			float sumSq{ 0.f };
			float count{ -1.f }; // assume dof lost in computing ave
			for (FwdIter iter{beg} ; end != iter ; ++iter)
			{
				sumSq += sq(*iter - ave);
				count += 1.f;
			}
			if (0.f < count)
			{
				var = sumSq / count;
			}
			return var;
		}

		template <typename FwdIter>
		inline
		explicit
		VecStats
			( FwdIter const & beg
			, FwdIter const & end
			)
			: theAve{ averageOf(beg, end) }
			, theVar{ varianceOf(beg, end, theAve) }
		{ }

	}; // VecStats


	//! A Larger grid produced with (nearest neighbor) up sampling.
	inline
	void
	foo
		( ras::Grid<float> const & srcGrid
		)
	{
		ras::Grid<float> sumGrid(srcGrid.hwSize());
		ras::Grid<float> rngGrid(srcGrid.hwSize());
		ras::Grid<float> aaGrid(srcGrid.hwSize());
		ras::Grid<float> avGrid(srcGrid.hwSize());
		ras::Grid<float> vaGrid(srcGrid.hwSize());
		ras::Grid<float> vvGrid(srcGrid.hwSize());
		std::fill(rngGrid.begin(), rngGrid.end(), -1.f);
		std::fill(aaGrid.begin(), aaGrid.end(),  0.f);
		std::fill(avGrid.begin(), avGrid.end(),  0.f);
		std::fill(vaGrid.begin(), vaGrid.end(),  0.f);
		std::fill(vvGrid.begin(), vvGrid.end(),  0.f);

		// boundary clipping
		img::Area const area
			{ img::Span{ 1., (double)(srcGrid.high() - 1u) }
			, img::Span{ 1., (double)(srcGrid.wide() - 1u) }
			};

//		std::vector<int> const winNdxs{ -4, -3, -2, -1, 0             };
		std::vector<int> const winNdxs{         -2, -1, 0             };
//		std::vector<int> const winNdxs{ -4, -3                        };
		float const fullCount{ (float)(winNdxs.size() * winNdxs.size()) };

constexpr bool showVals{ false };
		for (int row0{0u} ; row0 < (int)srcGrid.high() ; ++row0)
		{
			for (int col0{0u} ; col0 < (int)srcGrid.wide() ; ++col0)
			{
				float count{ 0.f };
				ops::Fence<float> aveRng;

				// anti-point difference should be small
				// variance (or range) of window values should be large
				// edge points should be included
				// edge mags should be anti-symmetric(ish)

				std::vector<float> aveVals;
				std::vector<float> varVals;
				aveVals.reserve(sq(winNdxs.size()));
				varVals.reserve(sq(winNdxs.size()));

				for (int const wRow : winNdxs)
				{
					double const dRowPos{ (double)(row0 + wRow) };
					double const dRowNeg{ (double)(row0 - wRow) };
					for (int const wCol : winNdxs)
					{
						double const dColPos{ (double)(col0 + wCol) };
						double const dColNeg{ (double)(col0 - wCol) };
						img::Spot const spotPos{ dRowPos, dColPos };
						img::Spot const spotNeg{ dRowNeg, dColNeg };
						if (area.contains(spotPos) && area.contains(spotNeg))
						{
							ras::RowCol const rcPos{ cast::rasRowCol(spotPos) };
							ras::RowCol const rcNeg{ cast::rasRowCol(spotNeg) };

							float const & valPos = srcGrid(rcPos);
							float const & valNeg = srcGrid(rcNeg);
							PairStats pairStat(valPos, valNeg);

							count += 1.f;

							aveRng.include(pairStat.theAve);

							aveVals.emplace_back(pairStat.theAve);
							varVals.emplace_back(pairStat.theVar);

if (showVals)
{
float const valAve{ pairStat.theAve };
float const valVar{ pairStat.theVar };
using engabra::g3::io::fixed;
std::cout << "\n";
std::cout << "valPos,Neg: " << fixed(valPos) << ' ' << fixed(valNeg) << '\n';
std::cout << "valAve,Var: " << fixed(valAve) << ' ' << fixed(valVar) << '\n';
}

						}
					}
				}

				if (fullCount <= count)
				{
					VecStats const aveStats{ aveVals.cbegin(), aveVals.cend() };
					float const aveAve{ aveStats.theAve };
					float const aveVar{ aveStats.theVar };
					VecStats const varStats{ varVals.cbegin(), varVals.cend() };
					float const varAve{ varStats.theAve };
					float const varVar{ varStats.theVar };

					ras::RowCol const rc0
						{ (std::size_t)row0, (std::size_t)col0 };
					rngGrid(rc0) = aveRng.max() - aveRng.min();
					aaGrid(rc0) = aveAve;
					avGrid(rc0) = aveVar;
					vaGrid(rc0) = varAve;
					vvGrid(rc0) = varVar;
if (showVals)
{
using engabra::g3::io::fixed;
std::cout << "aveAve,Var: " << fixed(aveAve) << ' ' << fixed(aveVar) << '\n';
std::cout << "varAve,Var: " << fixed(varAve) << ' ' << fixed(varVar) << '\n';
exit(8);
}
				}
			}
		}

		std::filesystem::path const srcPath("srcGrid.pgm");
		bool const srcOkay{ io::writeStretchPGM(srcPath, srcGrid) };
		std::cout << "srcOkay: " << srcOkay << '\n';

		std::filesystem::path const rngPath("rngGrid.pgm");
		bool const rngOkay{ io::writeStretchPGM(rngPath, rngGrid) };
		std::cout << "rngOkay: " << rngOkay << '\n';

		std::filesystem::path const aaPath("aaGrid.pgm");
		bool const aaOkay{ io::writeStretchPGM(aaPath, aaGrid) };
		std::cout << "aaOkay: " << aaOkay << '\n';

		std::filesystem::path const avPath("avGrid.pgm");
		bool const avOkay{ io::writeStretchPGM(avPath, avGrid) };
		std::cout << "avOkay: " << avOkay << '\n';

		std::filesystem::path const vaPath("vaGrid.pgm");
		bool const vaOkay{ io::writeStretchPGM(vaPath, vaGrid) };
		std::cout << "vaOkay: " << vaOkay << '\n';

		std::filesystem::path const vvPath("vvGrid.pgm");
		bool const vvOkay{ io::writeStretchPGM(vvPath, vvGrid) };
		std::cout << "vvOkay: " << vvOkay << '\n';

		// return
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
	std::filesystem::path const srcPath(argv[ndx++]);
	std::filesystem::path const outPath(argv[ndx++]);

	using namespace quadloco;

	// load image
	ras::Grid<std::uint8_t> const srcGrid{ io::readPGM(srcPath) };
	ras::Grid<float> const useGrid{ sig::util::toFloat(srcGrid) };

//	prb::Stats<float> const srcStats(useGrid.cbegin(), useGrid.cend());
//	std::cout << "\nsrcStats: " << srcStats << '\n';
	ras::Grid<img::Grad> const gradGrid{ ops::grid::gradientGridBy8x(useGrid) };
	ras::Grid<float> const magGrid{ sig::util::magnitudeGridFor(gradGrid) };
	foo(magGrid);

	constexpr std::size_t upFactor{ 1u };
	ras::Grid<float> outGrid{ sig::util::toLarger(useGrid, upFactor) };
	bool const okaySave{ io::writeStretchPGM(outPath, outGrid) };

	std::cout << '\n';
	std::cout << "src : " << srcPath << ' ' << srcGrid << '\n';
	std::cout << "out : " << outPath << ' ' << outGrid << '\n';
	std::cout << "okaySave: " << okaySave << '\n';
	std::cout << '\n';

	return 0;
}



