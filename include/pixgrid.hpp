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


#include "datChipSpec.hpp"
#include "datGrid.hpp"
#include "datSpot.hpp"
#include "pixGradel.hpp"
#include "pix.hpp"

#include <algorithm>
#include <cmath>
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

	//! Set pixels in full image that correspond with chip spec region
	template <typename Type>
	inline
	void
	setSubGridValues
		( dat::Grid<Type> * const & ptFull
		, dat::ChipSpec const & chipSpec
		, Type const & value
		)
	{
		for (std::size_t rowChip{0u} ; rowChip < chipSpec.high() ; ++rowChip)
		{
			for (std::size_t colChip{0u} ; colChip < chipSpec.wide()
				; ++colChip)
			{
				dat::RowCol const rcChip{ rowChip, colChip };
				(*ptFull)(chipSpec.rcFullForChipRC(rcChip)) = value;
			}
		}
	}

	//! Copy of fullGrid pixels defined by chip spec region
	template <typename Type>
	inline
	dat::Grid<Type>
	subGridValuesFrom
		( dat::Grid<Type> const & fullGrid
		, dat::ChipSpec const & chipSpec
		)
	{
		dat::Grid<Type> values(chipSpec.hwSize());
		for (std::size_t rowChip{0u} ; rowChip < chipSpec.high() ; ++rowChip)
		{
			for (std::size_t colChip{0u} ; colChip < chipSpec.wide()
				; ++colChip)
			{
				dat::RowCol const rcChip{ rowChip, colChip };
				dat::RowCol const rcFull{ chipSpec.rcFullForChipRC(rcChip) };
				values(rcChip) = fullGrid(rcFull);
			}
		}
		return values;
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

	/*! Value interpolated at location within grid using bilinear model.
	 *
	 * \note: Type must support operations including
	 * \arg (double * Type)
	 * \arg (Type +/- Type)
	 */
	template <typename Type>
	inline
	Type
	bilinValueAt
		( dat::Grid<Type> const & grid
		, dat::Spot const & at
		)
	{
		Type value{ pix::null<Type>() };

		// interpret 0,1 as row,col into grid
		double const & atRow = at[0];
		double const & atCol = at[1];

		// check if 'at' point is within grid at first (min) corner
		double const minRow{ std::floor(atRow) };
		double const minCol{ std::floor(atCol) };
		bool const isInMin{ (0. < minRow) && (0. < minCol) };
		if (isInMin)
		{
			// indices into grid (at minRow, minCol)
			std::size_t const ndxRow1{ static_cast<std::size_t>(atRow) };
			std::size_t const ndxCol1{ static_cast<std::size_t>(atCol) };
			std::size_t const ndxRow2{ ndxRow1 + 1u };
			std::size_t const ndxCol2{ ndxCol1 + 1u };

			// check if 'at' point is within grid at opposite (max) corner
			bool const isInMax
				{ (ndxRow2 < grid.high()) && (ndxCol2 < grid.wide()) };
			if (isInMax)
			{
				// fraction into the cell where interpolation is to occur
				double const rowFrac{ atRow - (double)(ndxRow1) };
				double const colFrac{ atCol - (double)(ndxCol1) };

				// Generate value spans at the four corners

				// interpolate along first (lower bounding) edge
				Type const val11{ grid(ndxRow1, ndxCol1) };
				Type const val21{ grid(ndxRow2, ndxCol1) };
				Type const val12{ grid(ndxRow1, ndxCol2) };
				Type const val22{ grid(ndxRow2, ndxCol2) };

				double const sclRow{ (atRow - minRow) };
				Type const dValA{ (Type)(sclRow * (val21 - val11)) };
				Type const dValB{ (Type)(sclRow * (val22 - val12)) };
				Type const valA{ val11 + dValA };
				Type const valB{ val12 + dValB };

				double const sclCol{ (atCol - minCol) };
				Type const dVal0{ (Type)(sclCol * (valB - valA)) };
				Type const val0{ dVal0 + valA };
				value = val0;
			}
		}

		return value;
	}


} // [grid]

} // [pix]

} // [quadloco]

