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
\brief Unit tests (and example) code for quadloco::ops::grid
*/


#include "QuadLoco/opsgrid.hpp"

#include "QuadLoco/rasChipSpec.hpp"
#include "QuadLoco/rasgrid.hpp"
#include "QuadLoco/rasGrid.hpp"
#include "QuadLoco/raskernel.hpp"
#include "QuadLoco/rasRowCol.hpp"
#include "QuadLoco/simgrid.hpp"

#include <algorithm>
#include <iostream>
#include <sstream>


namespace
{
	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		using namespace quadloco;

		// simulate grid with an sharp edge in it

		// simulate grid with strong edge consistent with expEdgel
		ras::SizeHW const hwSize{ 7u, 8u };
		std::vector<img::Edgel> const expEdgels
			{ img::Edgel
				{ img::Spot{ hwSize.centerSpot() }
				, img::Grad{  1., 0. }
				}
			, img::Edgel
				{ img::Spot{ hwSize.centerSpot() }
				, img::Grad{  0., 1. }
				}
			};
		// span hwSize *except* for *one* border pixel on each end of edge.
		// (one border cell is lost to gradient edge)
		std::vector<std::size_t> const expLinkSizes
			{ 2u * (hwSize.wide() - 2u) // for horizontal edge (spanning cols)
			, 2u * (hwSize.high() - 2u) // for vertical edge (spanning rows)
			};

		for (std::size_t ntest{0u} ; ntest < expEdgels.size() ; ++ntest)
		{

			// [DoxyExample01]

			// simulate grid with a (straight) step consisten with expEdgel.
			img::Edgel const expEdgel = expEdgels[ntest];
			ras::Grid<float> const pixGrid
				{ sim::gridWithEdge(hwSize, expEdgel, sim::Transition::Step) };
			ras::Grid<img::Grad> const gradGrid
				{ ops::grid::gradientGridBy8x(pixGrid) };

			// extract edgels from grid data

			// get edgel instances for all nonzero gradients
			std::vector<img::Edgel> const allEdgels
				{ ops::grid::allEdgelsFrom(gradGrid) };

			// get only the edgel instances strongly linked to adjacent ones
			std::vector<img::Edgel> const linkedEdgels
				{ ops::grid::linkedEdgelsFrom(gradGrid) };

			// [DoxyExample01]

			// for a step edge...
			// *ALL* of the detected gradient *magnitudes* should
			// be 1/2 of the expEdgel gradient magnitude.
			// (Since gradient is computed with grid stride of 2 cells)
			double const expGradMag{ .5 * expEdgel.magnitude() };
			using Iter = std::vector<img::Edgel>::const_iterator;
			std::pair<Iter, Iter> const iterMinMax
				{ std::minmax_element
					( allEdgels.cbegin(), allEdgels.cend()
					, [] (img::Edgel const & e1, img::Edgel const & e2)
						{ return (e1.magnitude() < e2.magnitude()); }
					)
				};
			double const gotGradMagMin{ iterMinMax.first->magnitude() };
			double const gotGradMagMax{ iterMinMax.second->magnitude() };

			if (! engabra::g3::nearlyEquals(gotGradMagMin, expGradMag))
			{
				oss << "Failure of (step) gradient gotGradMagMin test\n";
				oss << "exp: " << expGradMag << '\n';
				oss << "got: " << gotGradMagMin << '\n';
			}
			if (! engabra::g3::nearlyEquals(gotGradMagMax, expGradMag))
			{
				oss << "Failure of (step) gradient gotGradMagMax test\n";
				oss << "exp: " << expGradMag << '\n';
				oss << "got: " << gotGradMagMax << '\n';
			}

			// the strongly linked edges should span the grid format
			// *EXCEPT* for one border pixel on each end of the edge.
			if (! (linkedEdgels.size() == expLinkSizes[ntest]))
			{
				oss << "Failure of linkedEdgels.size() test\n";
				oss << "     ntest: " << ntest << '\n';
				oss << "  expEdgel: " << expEdgel << '\n';
				oss << "exp.size(): " << expLinkSizes[ntest] << '\n';
				oss << "got.size(): " << linkedEdgels.size() << '\n';
			}

			constexpr bool showData{ false };
			if (showData)
			{
				std::cout << pixGrid.infoStringContents
					("pixGrid", "{:5.2f}") << '\n';
				img::Vector<double>::Formatter fmtFunc{};
				std::cout << gradGrid.infoStringContents
					("gradGrid", fmtFunc) << '\n';
				std::cout << "\nallEdgel\n";
				for (img::Edgel const & edgel : allEdgels)
				{
					std::cout << "edgel: " << edgel
						<< ' ' << edgel.magnitude() << '\n';
				}
				std::cout << "\nlinkedEdgel\n";
				for (img::Edgel const & edgel : linkedEdgels)
				{
					std::cout << "edgel: " << edgel
						<< ' ' << edgel.magnitude() << '\n';
				}
				std::cout << "\nexpEdgel\n";
				std::cout << "edgel: " << expEdgel << '\n';
				std::cout << " eDir: " << expEdgel.direction() << '\n';
				std::cout << " eMag: " << expEdgel.magnitude() << '\n';
				std::cout << '\n';
			}

		} // individual image edge tests

		// check if linkEdgels is a subset of allEdgels
	}

	//! check computation of radient grids
	void
	test2
		( std::ostream & // oss
		)
	{
		// [DoxyExample02]

		// using tiny data grids for easy test data inspection
		constexpr std::size_t ndxHalf{ 4u };
		constexpr std::size_t ndxFull{ 2u*ndxHalf };
		quadloco::ras::SizeHW const hwSize{ ndxFull, ndxFull };
		// one with vertical, one with horizontal edges
		quadloco::ras::Grid<float> tbPixels(hwSize); // Top-to-Bot edge
		quadloco::ras::Grid<float> lrPixels(hwSize); // Lft-to-Rgt edge
		constexpr float backVal{ -15.25f };
		constexpr float foreVal{  -5.25f };
		// initialize both grids to background values
		std::fill(tbPixels.begin(), tbPixels.end(), backVal);
		std::fill(lrPixels.begin(), lrPixels.end(), backVal);

		// set foreground for bottom half of the horizontal edge grid
		std::fill(tbPixels.beginRow(ndxHalf), tbPixels.end(), foreVal);
		quadloco::img::Grad const tbExpGrad{ 10., 0. };

		// Use ChipSpec to set right half foreground for vertical edge grid
		quadloco::ras::ChipSpec const lrFillSpec
			{ quadloco::ras::RowCol{ 0u, lrPixels.wide()/2u }
			, quadloco::ras::SizeHW{ lrPixels.high(), lrPixels.wide()/2u }
			};
		quadloco::ras::grid::setSubGridValues(&lrPixels, lrFillSpec, foreVal);
		quadloco::img::Grad const lrExpGrad{ 0., 10. };

		// Compute edge gradient across all pixels

		// gradient magnitude prop to:
		quadloco::ras::Grid<quadloco::img::Grad> const lrGrads
			{ quadloco::ops::grid::gradientGridBy8x(lrPixels) };
		// Horizontal grid gradients
		quadloco::ras::Grid<quadloco::img::Grad> const tbGrads
			{ quadloco::ops::grid::gradientGridBy8x(tbPixels) };

		// [DoxyExample02]
	}

	//! Check magnitude and angle grid extraction
	void
	test3
		( std::ostream & oss
		)
	{
		// [DoxyExample03]

		using namespace quadloco;

		// create grid with "raised" square in middle
		ras::Grid<float> fullGrid{ ras::SizeHW{ 16u, 16u } };
		std::fill(fullGrid.begin(), fullGrid.end(), 0.f);
		ras::ChipSpec const chip
			{ ras::RowCol{ 4u, 4u }
			, ras::SizeHW{ 8u, 8u }
			};
		ras::grid::setSubGridValues(&fullGrid, chip, 2.f);

		// compute gradient grid
		ras::Grid<img::Grad> const gradGrid
			{ ops::grid::gradientGridBy8x(fullGrid) };

		// extract edgels from this grid
		std::vector<img::Edgel> const edgels
			{ ops::grid::allEdgelsFrom(gradGrid) };

		// generate grid with (all) edgel magnitudes
		ras::Grid<float> const magGrid
			{ ops::grid::edgeMagGridFor
				(gradGrid.hwSize(), edgels, edgels.size())
			};

		// generate grid with (all) edgel angles
		float const expBias{ -8.f }; // so angle values [-pi,pi) stand out
		ras::Grid<float> const angGrid
			{ ops::grid::edgeAngleGridFor
				(gradGrid.hwSize(), edgels, edgels.size(), expBias)
			};

		// [DoxyExample03]

		// expect number of non-zero gradients
		std::size_t const expNumGrad
			{ 4u * (chip.hwSize().high() - 2u) // gradients span two cells
			+ 4u * (chip.hwSize().wide() - 2u)
		//	+ 4u * 3u  // Cardinal Edges: each corner has 3 non zero gradients
			+ 4u * 4u  // 8-neighbor Edges: each corner has 4 non zero gradients
			};

		std::size_t const gotNumMag
			{ (std::size_t)std::count_if
				( magGrid.cbegin(), magGrid.cend()
				, [] (float const & val) { return (0.f < val); }
				)
			};
		std::size_t const gotNumAng
			{ (std::size_t)std::count_if
				( angGrid.cbegin(), angGrid.cend()
				, [& expBias] (float const & val) { return (expBias < val); }
				)
			};
		if ( (! (edgels.size() == expNumGrad))
		  || (! (gotNumMag == expNumGrad))
		  || (! (gotNumAng == expNumGrad))
		   )
		{
			oss << "Failure of (non-zero) gradient size test\n";
			oss << "exp: " << expNumGrad << '\n';
			oss << "   edgels: " << edgels.size() << '\n';
			oss << "gotNumMag: " << gotNumMag << '\n';
			oss << "gotNumAng: " << gotNumAng << '\n';
		}

		/*
		std::cout << fullGrid.infoStringContents("fullGrid", "%5.2f") << '\n';
		img::Vector<double>::Formatter const gradFmt{ "{:4.1f}" };
		std::cout << gradGrid.infoStringContents("gradGrid", gradFmt) << '\n';
		std::cout << magGrid.infoStringContents("magGrid", "%5.2f") << '\n';
		std::cout << angGrid.infoStringContents("angGrid", "%5.2f") << '\n';
		*/

	}

	//! check generic digital filtering operation
	void
	test4
		( std::ostream & oss
		)
	{
		// [DoxyExample04]

		using namespace quadloco;

		// simulate a data grid of values (impulse function)
		// filter (5x7) below is exact fit into this
		// with total number of nan filled rows/columns of 4 and 6
		ras::SizeHW const hwValues{ 5u+4u, 7u+6u };
		ras::RowCol const rcImpulse{ hwValues.high()/2u, hwValues.wide()/2u };
		ras::Grid<double> srcValues(hwValues);
		std::fill(srcValues.begin(), srcValues.end(), 0.);
		srcValues(rcImpulse) = 1.;

		// define a filter grid to apply to source image
		// Note filtering assume an odd size on each dimension
		std::size_t const halfHigh{ 2u }; // no result this many rows each edge
		std::size_t const halfWide{ 3u }; // no result this many cols each edge
		ras::SizeHW const hwFilter
			{ (2u*halfHigh + 1u)  // e.g. 5u
			, (2u*halfWide + 1u)  // e.g. 7u
			};
		ras::Grid<double> expFilter(hwFilter);
		std::iota(expFilter.begin(), expFilter.end(), 10.); // arbitrary

		// run filter across image
		ras::Grid<double> const gotValues
			{ ops::grid::filtered(srcValues, expFilter) };

		/*
		std::cout
			<< srcValues.infoStringContents("srcValues:", "%5.1f")
			<< '\n'
			<< expFilter.infoStringContents("expFilter:", "%5.1f")
			<< '\n'
			<< gotValues.infoStringContents("gotValues:", "%5.1f")
			<< '\n'
			;
		*/

		// filtered impulse function produces point reflected filter values
		// values in the output grid.
		ras::ChipSpec const responseChip
			{ ras::RowCol
				{ (rcImpulse.row() - (expFilter.high()/2u))
				, (rcImpulse.col() - (expFilter.wide()/2u))
				}
			, ras::SizeHW{ expFilter.hwSize() }
			};
		ras::Grid<double> const gotFilter // impulse response
			{ ras::grid::subGridValuesFrom<double>(gotValues, responseChip) };

		// [DoxyExample04]

		// For the impulse test data, response output is point reflection
		// reversed. Therefore, reverse the 'exp' values before testing.
		std::reverse(expFilter.begin(), expFilter.end());
		if (! nearlyEqualsAbs(gotFilter, expFilter))
		{
			oss << "Failure of gotFilter impulse response test\n";
			oss << "responseChip: " << responseChip << '\n';
			oss << expFilter.infoStringContents("exp:", "%5.2f") << '\n';
			oss << gotFilter.infoStringContents("got:", "%5.2f") << '\n';
		}
	}

	//! check smoothing filter
	void
	test5
		( std::ostream & oss
		)
	{
		// [DoxyExample05]

		using namespace quadloco;

		// construct a Gaussian filter window
		std::size_t const halfSize{ 3u };
		constexpr double sigma{ 1.25 };
		ras::Grid<float> const gFilter
			{ ras::kernel::gauss<float>(halfSize, sigma) };

		// integral of filter weights should be unity
		float const expSum{ 1.f };
		float const gotSum
			{ std::accumulate(gFilter.cbegin(), gFilter.cend(), 0.f) };

		// [DoxyExample05]

		// check if filter has unit integral
		float const tol{ 4.f*std::numeric_limits<float>::epsilon() };
		if (! engabra::g3::nearlyEquals(gotSum, expSum, tol))
		{
			using namespace engabra::g3::io;
			oss << "Failure of filter unit integral test\n";
			oss << "exp: " << fixed(expSum) << '\n';
			oss << "got: " << fixed(gotSum) << '\n';
			oss << "dif: " << enote(gotSum-expSum) << '\n';
		}
	}

	//! Check sub grid gradients
	void
	test6
		( std::ostream & oss
		)
	{
		// [DoxyExample06]

		using namespace quadloco;

		// fill grid with background values
		ras::SizeHW const hwSize{ 15u, 16u };
		ras::Grid<float> srcGrid{ hwSize };
		std::fill(srcGrid.begin(), srcGrid.end(), 0.f);

		// set a 'spike' value near middle
		constexpr std::size_t srcRow0{ 6u };
		constexpr std::size_t srcCol0{ 5u };
		srcGrid(srcRow0, srcCol0) = 1.f;

		// get gradient values from small sub area

		// define sub area
		std::size_t const halfChip{ 2u };
		ras::ChipSpec const chipSpec
			{ ras::RowCol{ srcRow0 - halfChip, srcCol0 - halfChip }
			, ras::SizeHW{ 5u, 5u }
			};

		// compute gradients
		ras::Grid<img::Grad> const gotGrad
			{ ops::grid::gradientSubGridBy8x(srcGrid, chipSpec) };

		// transform gradients to magnitudes for easy testing below
		ras::Grid<float> gotGMag{ gotGrad.hwSize() };
		ras::Grid<float>::iterator itGMag{ gotGMag.begin() };
		for (ras::Grid<img::Grad>::const_iterator
			itGrad{gotGrad.cbegin()} ; gotGrad.cend() != itGrad
			; ++itGrad, ++itGMag)
		{
			*itGMag = magnitude(*itGrad);
		}

		// expect gradient value in all 8 neighbors
		ras::Grid<float> expGMag{ chipSpec.hwSize() };
		std::fill(expGMag.begin(), expGMag.end(), 0.f);
		double const ir2{ 1. / std::sqrt(2.) };
		double const scl{ 1. / (4.*ir2 + 2.*1.) };
		std::size_t const chpRow0{ chipSpec.high() / 2u };
		std::size_t const chpCol0{ chipSpec.wide() / 2u };
		//
		expGMag(chpRow0-1u, chpCol0-1u) = scl * 1.;
		expGMag(chpRow0-1u, chpCol0   ) = scl * 1.;
		expGMag(chpRow0-1u, chpCol0+1u) = scl * 1.;
		//
		expGMag(chpRow0   , chpCol0-1u) = scl * 1.;
		expGMag(chpRow0   , chpCol0   ) = scl * 0.;
		expGMag(chpRow0   , chpCol0+1u) = scl * 1.;
		//
		expGMag(chpRow0+1u, chpCol0-1u) = scl * 1.;
		expGMag(chpRow0+1u, chpCol0   ) = scl * 1.;
		expGMag(chpRow0+1u, chpCol0+1u) = scl * 1.;

		// [DoxyExample06]

		if (! nearlyEquals(gotGMag, expGMag))
		{
			oss << "Failure of gotGMag nearlyEquals test\n";
			img::Vector<double>::Formatter const fmtr{ "%6.3f" };
		//	oss << srcGrid.infoStringContents("srcGrid", "%6.3f") << '\n';
			oss << gotGrad.infoStringContents("gotGrad", fmtr, "  ") << '\n';
			oss << expGMag.infoStringContents("expGMag", "%6.3f") << '\n';
			oss << gotGMag.infoStringContents("gotGMag", "%6.3f") << '\n';
			oss << '\n';
			oss << "chipSpec: " << chipSpec << '\n';
			oss << " expGMag: " << expGMag << '\n';
			oss << " gotGrad: " << gotGrad << '\n';
			oss << " gotGMag: " << gotGMag << '\n';
			oss << '\n';
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
	test2(oss);
	test3(oss);
	test4(oss);
	test5(oss);
	test6(oss);

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

