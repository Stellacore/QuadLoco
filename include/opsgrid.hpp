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


#pragma once


/*! \file
 * \brief Top level file for quadloco::NM namespace
 *
 */


#include "imgEdgel.hpp"
#include "imgGrad.hpp"
#include "rasgrid.hpp"
#include "rasGrid.hpp"
#include "rasRowCol.hpp"
#include "rasSizeHW.hpp"

#include <algorithm>
#include <limits>
#include <vector>


namespace quadloco
{

/*! \brief Namespaced functions and utilities for quadloco::ops::EdgelGetter
 */
namespace ops
{

namespace grid
{
	/*! Compute img::Grad for each pixel location (except at edges)
	 *
	 * The stepHalf determine how wide an increment is used to estimate
	 * the edge in each direction. E.g., for a one dimensional signal
	 * array 'gVal', the gradient at location ndx is computed as:
	 * \verbatim
	 * grad1D = (gVal[ndx+stepHalf] - gVal[ndx-stepHalf]) / (2.*stepHalf)
	 * \endverbatim
	 *
	 * For 2D grid, the img::Grad components are computed similarly for
	 * each direction. Note that this is NOT a classic window computation
	 * but rather two indpenendent evaluations done in each index. E.g.,
	 * at location (row,col), the computed gradient is:
	 * \verbatim
	 * del = 2. * (double)stepHalf;
	 * gradValue[0] = (gVal(row+stepHalf, col) - gval(row-stepHalf,col)) / del
	 * gradValue[1] = (gVal(row, col+stepHalf) - gval(row,col-stepHalf)) / del
	 * \endverbatim
	 *
	 * NOTE: the stepHalf pixels around the boarder are set to null!!
	 */
	inline
	ras::Grid<img::Grad>
	gradientGridFor
		( ras::Grid<float> const & inGrid
		, std::size_t const & stepHalf = 1u
		)
	{
		ras::Grid<img::Grad> grads;

		// check if there is enough room to process anything
		std::size_t const stepFull{ 2u * stepHalf };
		if ((stepFull < inGrid.high()) && (stepFull < inGrid.wide()))
		{
			// allocate space
			ras::SizeHW const hwSize{ inGrid.hwSize() };
			grads = ras::Grid<img::Grad>(hwSize);

			//! Determine start and end indices
			std::size_t const rowNdxBeg{ stepHalf };
			std::size_t const rowNdxEnd{ rowNdxBeg + hwSize.high() - stepFull };
			std::size_t const colNdxBeg{ stepHalf };
			std::size_t const colNdxEnd{ colNdxBeg + hwSize.wide() - stepFull };

			// set border to null values
			static img::Grad const gNull{};
			ras::grid::fillBorder
				(grads.begin(), grads.hwSize(), stepHalf, gNull);

			float const scl{ 1.f / (float)stepFull };
			for (std::size_t row{rowNdxBeg} ; row < rowNdxEnd ; ++row)
			{
				std::size_t const rowM1{ row - stepHalf };
				std::size_t const rowP1{ row + stepHalf };
				for (std::size_t col{colNdxBeg} ; col < colNdxEnd ; ++col)
				{
					std::size_t const colM1{ col - stepHalf };
					std::size_t const colP1{ col + stepHalf };
					float const rowGrad
						{ scl * (inGrid(rowP1, col) - inGrid(rowM1, col)) };
					float const colGrad
						{ scl * (inGrid(row, colP1) - inGrid(row, colM1)) };

					grads(row,col) = img::Grad{ rowGrad, colGrad };
				}
			}
		}

		return grads;
	}


	//! Collection of all edgels associated with non-zero gradients.
	inline
	std::vector<img::Edgel>
	allEdgelsFrom
		( ras::Grid<img::Grad> const & gradGrid
			//!< Gradient grid from which to extract edgels (non-zero grads)
		)
	{
		std::vector<img::Edgel> pixEdgels{};
		pixEdgels.reserve(gradGrid.size());

		// could skip unset edge rows
		for (std::size_t row{0u} ; row < gradGrid.high() ; ++row)
		{
			for (std::size_t col{0u} ; col < gradGrid.wide() ; ++col)
			{
				img::Grad const & grad = gradGrid(row, col);
				if (grad.isValid())
				{
					constexpr double tol
						{ std::numeric_limits<double>::epsilon() };
					if (tol < magnitude(grad))
					{
						// should be valid since grad is significant
						img::Edgel const edgel(ras::RowCol{ row, col }, grad);
						pixEdgels.push_back(edgel);
					}
				}
			}
		}

		return pixEdgels;
	}

	//! Functor for checking if a (row,col) location is inside padded boundary
	struct Inside
	{
		std::size_t theBegRow{ 0u };
		std::size_t theBegCol{ 0u };
		std::size_t theEndRow{ 0u };
		std::size_t theEndCol{ 0u };

		//! Construct to test inside hwSize within 'pad' cell border all around
		inline
		explicit
		Inside
			( ras::SizeHW const & hwSize
			, std::size_t const & pad
			)
		{
			bool const highNuf{ (2u*pad) < hwSize.high() };
			bool const wideNuf{ (2u*pad) < hwSize.wide() };
			if (highNuf && wideNuf)
			{
				theBegRow = pad;
				theBegCol = pad;
				theEndRow = hwSize.high() - pad;
				theEndCol = hwSize.wide() - pad;
			}
		}

		//! True if row,col is inside hwSize accounting for border pad border
		inline
		bool
		operator()
			( std::size_t const & row
			, std::size_t const & col
			) const
		{
			return
				(  (theBegRow < row) && (row < theEndRow)
				&& (theBegCol < col) && (col < theEndCol)
				);
		}

	};

	//! Collection edgels that have corroborating neighbors
	inline
	std::vector<img::Edgel>
	linkedEdgelsFrom
		( ras::Grid<img::Grad> const & gradGrid
			//!< Gradient grid from which to extract edgels (non-zero grads)
		, double const & supportMultiplier = 2.5
			//!< Degree of support (1: self only, 2: one other, etc)
		)
	{
		std::vector<img::Edgel> pixEdgels{};
		std::size_t const numElem{ gradGrid.size() };
		if (0u < numElem)
		{
			pixEdgels.reserve(numElem);

			// testing
			Inside const inFull(gradGrid.hwSize(), 2u);
			Inside const inEdge(gradGrid.hwSize(), 1u);

			std::size_t const rowLast{ gradGrid.high() - 1u };
			std::size_t const colLast{ gradGrid.wide() - 1u };

			// leave room to compute with neighbors
			// could skip unset edge rows
			for (std::size_t row{1u} ; row < rowLast ; ++row)
			{
				for (std::size_t col{1u} ; col < colLast ; ++col)
				{
					// adjust tolerance at edge boundary
					double minProjDist{ supportMultiplier };
					if (! inFull(row, col))
					{
						if (inEdge(row, col))
						{
							minProjDist *= .5;
						}
					}

					// gradient at center
					img::Grad const & gradCenter = gradGrid(row, col);
					if (gradCenter.isValid())
					{
						double const gMag{ magnitude(gradCenter) };
						constexpr double tol
							{ std::numeric_limits<double>::epsilon() };
						if (tol < gMag)
						{
							// sum gradients for neighbor hood
							img::Grad const gradHoodSum
								{ gradGrid(row - 1u, col - 1u)
								+ gradGrid(row - 1u, col     )
								+ gradGrid(row - 1u, col + 1u)
								+ gradGrid(row     , col - 1u)
								+ gradGrid(row     , col + 1u)
								+ gradGrid(row + 1u, col - 1u)
								+ gradGrid(row + 1u, col     )
								+ gradGrid(row + 1u, col + 1u)
								};
							if (gradHoodSum.isValid())
							{
								img::Grad const gradSum
									{ gradHoodSum + gradCenter };
								img::Grad const gDir{ gradCenter };
								double const projDist
									{ (1./gMag) * dot(gradSum, gDir) };

								if (minProjDist < projDist)
								{
									// assured to be valid at this point since
									// only processing non-trivial gradients
									// i.e., tol < magnitude(gradCenter) above
									img::Edgel const edgel
										(ras::RowCol{ row, col }, gradCenter);
									pixEdgels.push_back(edgel);
								}
							}
						}
					}
				}
			}
		}

		return pixEdgels;
	}

	//! Populate a grid with edgels magnitude data (only at edgel locations)
	inline
	ras::Grid<float>
	edgeMagGridFor
		( ras::SizeHW const & hwSize
		, std::vector<img::Edgel> const & edgels
		, std::size_t const & numToUse
		)
	{
		ras::Grid<float> grid(hwSize);
		std::fill(grid.begin(), grid.end(), 0.f);
		for (std::size_t nn{0u} ; nn < numToUse ; ++nn)
		{
			img::Edgel const & edgel = edgels[nn];
			if (! isValid(edgel))
			{
				std::cerr << "ERROR got invalid edgel were not expecting it\n";
				exit(9);
			}
			ras::RowCol const rc{ cast::rasRowCol(edgel.location()) };
			grid(rc) = edgel.magnitude();
		}

		return grid;
	}

	//! Populate a grid with edgels angles data (only at edgel locations)
	inline
	ras::Grid<float>
	edgeAngleGridFor
		( ras::SizeHW const & hwSize
		, std::vector<img::Edgel> const & edgels
		, std::size_t const & numToUse
		, float const & backgroundBias = -5.f
		)
	{
		ras::Grid<float> grid(hwSize);
		// angle values range from [-pi, pi), so start with a background
		// bias so that angle values are better distinguished.
		std::fill(grid.begin(), grid.end(), backgroundBias);
		for (std::size_t nn{0u} ; nn < numToUse ; ++nn)
		{
			img::Edgel const & edgel = edgels[nn];
			if (! isValid(edgel))
			{
				std::cerr << "ERROR got invalid edgel were not expecting it\n";
				exit(9);
			}
			ras::RowCol const rc{ cast::rasRowCol(edgel.location()) };
			double const angle{ edgel.angle() };
			grid(rc) = (float)angle;
		}

		return grid;
	}


} // grid

} // [ops]

} // [quadloco]

