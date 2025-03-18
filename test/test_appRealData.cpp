//
// MIT License
//
// Copyright (c) 2025 Stellacore Corporation
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
\brief Unit tests (and example) code for quadloco::app with actual data
*/


#include "QuadLoco/QuadLoco"

#include <Engabra>

#include <array>
#include <iostream>
#include <sstream>
#include <utility>


namespace
{
	//! Test processing actual data samples
	std::vector<quadloco::ras::PeakRCV>
	symRingPeaksFrom
		( std::filesystem::path const & srcPath
		)
	{
		// [DoxyExample01]

		using namespace quadloco;

		// load test data
		ras::Grid<uint8_t> const pgmGrid{ io::readPGM(srcPath) };

		// cast to floating point grid
		ras::Grid<float> const srcGrid
			{ ras::grid::realGridOf<float>(pgmGrid) };

		// get local source grid statistics
		prb::Stats<float> const srcStats(srcGrid.cbegin(), srcGrid.cend());

		// get peaks in symmetry ring filter response
		std::vector<std::size_t> const ringHalfs{ 5u, 3u };
		std::vector<ras::PeakRCV> const peaks
			{ app::center::multiSymRingPeaks(srcGrid, srcStats, ringHalfs) };

		// [DoxyExample01]

		return peaks;
	}

	//! Put information about peaks to ostrm
	inline
	void
	showPeaks
		( std::ostream & ostrm
		, std::vector<quadloco::ras::PeakRCV> const & peaks
		)
	{
		std::size_t const tryShow{ 3u };
		std::size_t const numShow
			{ std::min<std::size_t>(tryShow, peaks.size()) };
		ostrm << "peaks.size: " << peaks.size() << '\n';
		for (std::size_t nn{0u} ; nn < numShow ; ++nn)
		{
			quadloco::ras::PeakRCV const & peak = peaks[nn];
			using engabra::g3::io::fixed;
			ostrm
				<< "... peak[" << nn << "]:"
				<< ' ' << peak.rcLocation()
				<< ' ' << fixed(peak.value(), 2u, 9u)
				<< '\n';
		}
	}

	//! Test processing actual data samples
	void
	test1
		( std::ostream & oss
		)
	{
		using namespace quadloco;

		// configure real data trial cases
		using PathSpot = std::pair<std::filesystem::path, img::Spot>;
		constexpr std::size_t numTrials{ 2u };
		// data/filenames and expected quad center loc (from manual measure)
		std::array<PathSpot, numTrials> const expPathSpots
			{ PathSpot{ "./p5q5.pgm", img::Spot{ 24.39, 25.05 } }
			, PathSpot{ "./p5q6.pgm", img::Spot{ 25.06, 24.95 } }
			};

		// check each trial case
		std::size_t const numTest{ expPathSpots.size() };
		for (std::size_t nn{0u} ; nn < numTest ; ++nn)
		{
			std::filesystem::path const & srcPath = expPathSpots[nn].first;
			img::Spot const & expSpot = expPathSpots[nn].second;

			std::vector<ras::PeakRCV> const allPeaks
				{ symRingPeaksFrom(srcPath) };

			constexpr bool showPeakInfo{ false };
			if (showPeakInfo)
			{
				std::cout << '\n';
				std::cout << "srcPath: " << srcPath << '\n';
				showPeaks(std::cout, allPeaks);
				std::cout << '\n';
			}

			if ( allPeaks.empty())
			{
				oss << "Failure of non-empty peaks test\n";
				oss << "srcPath: " << srcPath << '\n';
			}
			else
			{
				ras::PeakRCV const & gotPeak = allPeaks.front();
				if (! gotPeak.isValid())
				{
					oss << "Failure to get best peak\n";
					oss << "srcPath: " << srcPath << '\n';
					oss << "gotPeak: " << gotPeak << '\n';
				}
				else
				{
					img::Spot const gotSpot
						{ cast::imgSpot(gotPeak.rcLocation()) };
					constexpr double tol{ 1.00 }; // in each coordinate
					if (! nearlyEqualsAbs(gotSpot, expSpot, tol))
					{
						img::Spot const difSpot{ gotSpot - expSpot };
						double const difMag{ magnitude(difSpot) };
						oss << "Failure of best peak location test\n";
						oss << "srcPath: " << srcPath << '\n';
						oss << "expSpot: " <<  expSpot << '\n';
						oss << "gotSpot: " << gotSpot << '\n';
						oss << "difSpot: " << difSpot << '\n';
						oss << " difMag: " << difMag << '\n';
					}
				}
			}
		}

	}

}

//! Standard test case main wrapper
int
main
	()
{
	int status{ 1 };
	std::stringstream oss;

//	test0(oss);
	test1(oss);

	if (oss.str().empty()) // Only pass if no errors were encountered
	{
		status = 0;
	}
	else
	{
		// else report error messages
		std::cerr << "### FAILURE in test file: " << __FILE__ << std::endl;
		std::cerr << oss.str();
	}
	return status;
}

