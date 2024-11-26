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

	//! Collection edgels that have corroborating neighbors
	inline
	std::vector<img::Edgel>
	linkedEdgelsFrom
		( ras::Grid<img::Grad> const & rasGradGrid
			//!< Gradient grid from which to extract edgels (non-zero grads)
		, double const & supportMultipler = 2.5
			//!< Degree of support (1: self only, 2: one other, etc)
		)
	{
		return {};
	}

} // grid

} // [ops]

} // [quadloco]

