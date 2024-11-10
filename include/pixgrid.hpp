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
 * \brief Declarations for quadloco::pix::grid:: processing functions
 *
 */


#include "datGrid.hpp"
#include "pixGradel.hpp"

#include <algorithm>
#include <utility>


namespace quadloco
{

namespace pix
{

/*! \brief Pixel grid processing functions (e.g., for image processing)
 */
namespace grid
{
	//! Put value into the *FIRST* nRows after beg assuming dimensions hwSize
	template <typename FwdIter, typename Type>
	inline
	void
	fillInitRows
		( FwdIter const & beg
		, dat::SizeHW const & hwSize
		, std::size_t const & nRows
		, Type const & value
		)
	{
		std::fill
			( beg
			, beg + nRows * hwSize.wide()
			, value
			);
	}

	//! Put value into the *LAST* nRows assuming dimensions hwSize from beg
	template <typename FwdIter, typename Type>
	inline
	void
	fillLastRows
		( FwdIter const & beg
		, dat::SizeHW const & hwSize
		, std::size_t const & nCols
		, Type const & value
		)
	{
		std::fill
			( beg + hwSize.size() - nCols * hwSize.wide()
			, beg + hwSize.size()
			, value
			);
	}

	//! Put value into the *FIRST* nCols from beg assuming hwSize
	template <typename FwdIter, typename Type>
	inline
	void
	fillInitCols
		( FwdIter const & beg
		, dat::SizeHW const & hwSize
		, std::size_t const & nCols
		, Type const & value
		)
	{
		for (std::size_t row{0u} ; row < hwSize.high() ; ++row)
		{
			FwdIter const colBeg{ beg + (row * hwSize.wide()) };
			FwdIter const colEnd{ colBeg + nCols };
			std::fill(colBeg, colEnd, value);
		}
	}

	//! Put value into the *LAST* nCols assuming dimensions hwSize from beg
	template <typename FwdIter, typename Type>
	inline
	void
	fillLastCols
		( FwdIter const & beg
		, dat::SizeHW const & hwSize
		, std::size_t const & nCols
		, Type const & value
		)
	{
		std::size_t const colOffset{ hwSize.wide() - nCols };
		for (std::size_t row{0u} ; row < hwSize.high() ; ++row)
		{
			FwdIter const colBeg{ beg + (row * hwSize.wide() + colOffset) };
			FwdIter const colEnd{ colBeg + nCols };
			std::fill(colBeg, colEnd, value);
		}
	}

	//! Put value into *BORDER* nPad thick assuming dimensions hwSize from beg
	template <typename FwdIter, typename Type>
	inline
	void
	fillBorder
		( FwdIter const & beg
		, dat::SizeHW const & hwSize
		, std::size_t const & nPad
		, Type const & value
		)
	{
		//! fill initial rows
		fillInitRows(beg, hwSize, nPad, value);

		// fill columns (as contiguous values over end(row) and start(row+1)
		std::size_t const midSkip{ hwSize.wide() - 2*nPad };
		std::size_t const rowBeg{ nPad };
		std::size_t const rowEnd{ hwSize.high() - nPad };
		std::size_t const rowDelta{ hwSize.wide() };
		for (std::size_t row{nPad-1u} ; row < rowEnd ; ++row)
		{
			FwdIter const itBeg{ beg + (row*rowDelta) + nPad + midSkip };
			FwdIter const itEnd{ itBeg + 2*nPad };
			std::fill(itBeg, itEnd, value);
		}

		//! fill final rows
		fillLastRows(beg, hwSize, nPad, value);
	}


	/*! Compute Gradels for each pixel location (except at edges)
	 *
	 * The stepHalf determine how wide an increment is used to estimate
	 * the edge in each direction. E.g., for a one dimensional signal
	 * array 'gVal', the gradient at location ndx is computed as:
	 * \verbatim
	 * grad1D = (gVal[ndx+stepHalf] - gVal[ndx-stepHalf]) / (2.*stepHalf)
	 * \endverbatim
	 *
	 * For 2D grid, the Gradel components are computed similarly for
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
	static
	dat::Grid<pix::Gradel>
	gradelGridFor
		( dat::Grid<float> const & inGrid
		, std::size_t const & stepHalf = 1u
		)
	{
		dat::Grid<pix::Gradel> gradels;

		// check if there is enough room to process anything
		std::size_t const stepFull{ 2u * stepHalf };
		if ((stepFull < inGrid.high()) && (stepFull < inGrid.wide()))
		{
			// allocate space
			dat::SizeHW const hwSize{ inGrid.hwSize() };
			gradels = dat::Grid<pix::Gradel>(hwSize);

			//! Determine start and end indices
			std::size_t const rowNdxBeg{ stepHalf };
			std::size_t const rowNdxEnd{ rowNdxBeg + hwSize.high() - stepFull };
			std::size_t const colNdxBeg{ stepHalf };
			std::size_t const colNdxEnd{ colNdxBeg + hwSize.wide() - stepFull };

			// set border to null values
			static Gradel const gNull{};
			fillBorder(gradels.begin(), gradels.hwSize(), stepHalf, gNull);

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

					gradels(row,col) = Gradel{ rowGrad, colGrad };
				}
			}
		}

		return gradels;
	}


} // [grid]

} // [pix]

} // [quadloco]

