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


#include "QuadLoco/pix.hpp"

#include "QuadLoco/imgArea.hpp"
#include "QuadLoco/imgGrad.hpp"
#include "QuadLoco/imgSpot.hpp"
#include "QuadLoco/pix.hpp"
#include "QuadLoco/rasChipSpec.hpp"
#include "QuadLoco/rasGrid.hpp"
#include "QuadLoco/rasRowCol.hpp"
#include "QuadLoco/valSpan.hpp"

#include <Engabra> // TODO - temp

#include <algorithm>
#include <cmath>
#include <utility>


namespace quadloco
{

namespace ras
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
		, ras::SizeHW const & hwSize
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
		, ras::SizeHW const & hwSize
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
		, ras::SizeHW const & hwSize
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
		, ras::SizeHW const & hwSize
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
		, ras::SizeHW const & hwSize
		, std::size_t const & nPad
		, Type const & value
		)
	{
		//! fill initial rows
		fillInitRows(beg, hwSize, nPad, value);

		// fill columns (as contiguous values over end(row) and start(row+1)
		std::size_t const midSkip{ hwSize.wide() - 2*nPad };
		std::size_t const rowBeg{ nPad - 1u};
		std::size_t const rowEnd{ hwSize.high() - nPad };
		std::size_t const rowDelta{ hwSize.wide() };
		for (std::size_t row{rowBeg} ; row < rowEnd ; ++row)
		{
			FwdIter const itBeg{ beg + (row*rowDelta) + nPad + midSkip };
			FwdIter const itEnd{ itBeg + 2*nPad };
			std::fill(itBeg, itEnd, value);
		}

		//! fill final rows
		fillLastRows(beg, hwSize, nPad, value);
	}

	//! Iterators to (not null) min/max *valid* elements in collection
	template <typename Iter>
	inline
	std::pair<Iter, Iter>
	minmax_valid
		( Iter const & beg
		, Iter const & end
		)
	{
		std::pair<Iter, Iter> itPair{ end, end };
		Iter & itMin = itPair.first;
		Iter & itMax = itPair.second;
		using ValType = typename std::iterator_traits<Iter>::value_type;
		ValType min{ pix::null<ValType>() };
		ValType max{ pix::null<ValType>() };
		for (Iter iter{beg} ; end != iter ; ++iter)
		{
			ValType const & value = *iter;
			if (pix::isValid(value))
			{
				bool setMin{ false };
				// track min
				if (pix::isValid(min))
				{
					setMin = (value < min);
				}
				else
				{
					setMin = true;
				}
				if (setMin)
				{
					min = value;
					itMin = iter;
				}

				// track max
				bool setMax{ false };
				if (pix::isValid(max))
				{
					setMax = (max < value);
				}
				else
				{
					setMax = true;
				}
				if (setMax)
				{
					max = value;
					itMax = iter;
				}
			}
		}
		return itPair;
	}

	//! Min/Max values from a collection with only isValid() inputs considered
	template <typename FwdIter, typename PixType = float>
	inline
	std::pair<PixType, PixType>
	validMinMaxValues
		( FwdIter const & beg
		, FwdIter const & end
		)
	{
		std::pair<PixType, PixType> minmax
			{ pix::null<PixType>(), pix::null<PixType>() };
		std::pair<FwdIter, FwdIter> const itPair{ minmax_valid(beg, end) };
		FwdIter const & itMin = itPair.first;
		FwdIter const & itMax = itPair.second;
		if ((end != itMin) && (end != itMax))
		{
			minmax.first = *itMin;
			minmax.second = *itMax;
		}
		return minmax;
	}

	//! Span with begin/end at smallest/largest (valid) values in fGrid
	template <typename PixType>
	inline
	val::Span
	fullSpanFor
		( ras::Grid<PixType> const & fGrid
		)
	{
		std::pair<PixType, PixType> const fMinMax
			{ validMinMaxValues(fGrid.cbegin(), fGrid.cend()) };
		PixType const & fMin = fMinMax.first;
		// bump max by a tiny amount so that largest value *is* included
		PixType const & fMax = fMinMax.second;
		PixType const delta{ fMax - fMin };
		constexpr PixType eps{ std::numeric_limits<PixType>::epsilon() };
		PixType const useMax{ fMax * (1.f + delta * eps) };
		return quadloco::val::Span{ (double)fMin, (double)useMax };
	}

	//! \brief Convert grid elements to float
	template <typename PixType>
	inline
	ras::Grid<PixType>
	realGridOf
		( ras::Grid<uint8_t> const & uGrid
			//!< Input source grid
		, int const & treatAsNull = -1
			//!< If value in range [0,255], then corresponding output is null
		)
	{
		typename ras::Grid<PixType> fGrid(uGrid.hwSize());
		ras::Grid<uint8_t>::const_iterator inIter{ uGrid.cbegin() };
		typename ras::Grid<PixType>::iterator outIter{ fGrid.begin() };
		while (uGrid.cend() != inIter)
		{
			uint8_t const & inVal = *inIter;
			PixType rtVal{ static_cast<PixType>(inVal) };
			bool const checkForNull{ ! (treatAsNull < 0) };
			if (checkForNull)
			{
				if (treatAsNull == inVal)
				{
					rtVal = std::numeric_limits<PixType>::quiet_NaN();
				}
			}
			*outIter = rtVal;
			inIter++;
			outIter++;
		}
		return fGrid;
	}

	//! \brief A Larger grid produced with (nearest neighbor) up sampling.
	template <typename PixType>
	inline
	ras::Grid<PixType>
	largerGrid
		( ras::Grid<PixType> const & inGrid
		, std::size_t const & upFactor
		)
	{
		ras::Grid<PixType> upGrid
			( upFactor * inGrid.high()
			, upFactor * inGrid.wide()
			);
		for (std::size_t inRow{0u} ; inRow < inGrid.high() ; ++inRow)
		{
			std::size_t const outRow0{ upFactor * inRow };
			for (std::size_t inCol{0u} ; inCol < inGrid.wide() ; ++inCol)
			{
				std::size_t const outCol0{ upFactor * inCol };
				for (std::size_t wRow{0u} ; wRow < upFactor ; ++wRow)
				{
					std::size_t const outRow{ outRow0 + wRow };
					for (std::size_t wCol{0u} ; wCol < upFactor ; ++wCol)
					{
						std::size_t const outCol{ outCol0 + wCol };
						upGrid(outRow, outCol) = inGrid(inRow, inCol);
					}
				}
			}
		}
		return upGrid;
	}

	//! \brief Grid result of applying function to each input cell
	template <typename Func>
	inline
	ras::Grid<float>
	resultGridFor
		( ras::Grid<img::Grad> const & gradGrid
		, Func const & funcOfGrad
		)
	{
		ras::Grid<float> outGrid(gradGrid.hwSize());
		std::transform
			( gradGrid.cbegin(), gradGrid.cend()
			, outGrid.begin()
			, funcOfGrad
			);
		return outGrid;
	}

	/*! \brief A uint8_t grid that maps fgrid values via fSpan range.
	 *
	 * ref uPix8() for details on value mapping.
	 */
	inline
	ras::Grid<uint8_t>
	uGrid8
		( ras::Grid<float> const & fgrid
		, val::Span const & fSpan
		)
	{
		ras::Grid<uint8_t> ugrid{ fgrid.hwSize() };

		ras::Grid<float>::const_iterator itIn{ fgrid.cbegin() };
		ras::Grid<uint8_t>::iterator itOut{ ugrid.begin() };
		while (ugrid.end() != itOut)
		{
			*itOut++ = pix::uPix8(*itIn++, fSpan);
		}
		return ugrid;
	}

	//! Set pixels in full image that correspond with chip spec region
	template <typename Type>
	inline
	void
	setSubGridValues
		( ras::Grid<Type> * const & ptFull
		, ras::ChipSpec const & chipSpec
		, Type const & value
		)
	{
		std::size_t const cHigh{ chipSpec.high() };
		std::size_t const cWide{ chipSpec.wide() };
		for (std::size_t rowChip{0u} ; rowChip < cHigh ; ++rowChip)
		{
			for (std::size_t colChip{0u} ; colChip < cWide ; ++colChip)
			{
				ras::RowCol const rcChip{ rowChip, colChip };
				(*ptFull)(chipSpec.rcFullForChipRC(rcChip)) = value;
			}
		}
	}

	//! Populate ptChipData cells with values from fullData (True if possible)
	template <typename Type>
	inline
	bool
	setSubGridInside
		( ras::Grid<Type> * const ptFullData
		, ras::Grid<Type> const & chipData
		, ras::RowCol const & rc0
		)
	{
		bool okay{ false };
		ras::SizeHW const hwChip{ chipData.hwSize() };
		ras::ChipSpec const chipSpec{ rc0, hwChip };
		okay = ptFullData && chipSpec.fitsInto(ptFullData->hwSize());
		if (okay)
		{
			std::size_t const highChip{ hwChip.high() };
			std::size_t const wideChip{ hwChip.wide() };
			for (std::size_t rChip{0u} ; rChip < highChip ; ++rChip)
			{
				for (std::size_t cChip{0u} ; cChip < wideChip ; ++cChip)
				{
					ras::RowCol const rcChip{ rChip, cChip };
					ras::RowCol const rcFull
						{ chipSpec.rcFullForChipRC(rcChip) };
					(*ptFullData)(rcFull) = chipData(rcChip);
				}
			}
		}
		return okay;
	}

	//! Copy of fullGrid pixels defined by chip spec region
	template <typename OutType, typename SrcType>
	inline
	ras::Grid<OutType>
	subGridValuesFrom
		( ras::Grid<SrcType> const & fullGrid
		, ras::ChipSpec const & chipSpec
		)
	{
		ras::Grid<OutType> values;
		if (chipSpec.fitsInto(fullGrid.hwSize()))
		{
			std::size_t const high{ chipSpec.high() };
			std::size_t const wide{ chipSpec.wide() };
			values = ras::Grid<OutType>(chipSpec.hwSize());
			for (std::size_t rowChip{0u} ; rowChip < high ; ++rowChip)
			{
				for (std::size_t colChip{0u} ; colChip < wide ; ++colChip)
				{
					ras::RowCol const rcChip{ rowChip, colChip };
					ras::RowCol const rcFull
						{ chipSpec.rcFullForChipRC(rcChip) };
					values(rcChip) = static_cast<OutType>(fullGrid(rcFull));
				}
			}
		}
		return values;
	}

	//! Draw a (bright on black) radiometric mark at spot
	template <typename PixType>
	inline
	void
	drawSpot
		( ras::Grid<PixType> * const & ptGrid
		, img::Spot const & spot
		)
	{
		if (ptGrid && (2u < ptGrid->high()) && (2u < ptGrid->wide()))
		{
			ras::Grid<PixType> & grid = *ptGrid;

			// find min max
			using InIt = typename ras::Grid<PixType>::const_iterator;
			std::pair<InIt, InIt> const iterMM
				{ minmax_valid(grid.cbegin(), grid.cend()) };
			PixType const min{ *(iterMM.first) };
			PixType const max{ *(iterMM.second) };

			// boundary clipping
			img::Area const clipArea
				{ val::Span{ 1., (double)(ptGrid->high() - 1u) }
				, val::Span{ 1., (double)(ptGrid->wide() - 1u) }
				};

			PixType const & back = min;
			PixType const & fore = max;
			if (clipArea.contains(spot))
			{
				ras::RowCol const rc0
					{ static_cast<std::size_t>(std::floor(spot[0]))
					, static_cast<std::size_t>(std::floor(spot[1]))
					};
				grid(rc0.row() - 1u, rc0.col() - 1u) = back;
				grid(rc0.row() - 1u, rc0.col()     ) = back;
				grid(rc0.row() - 1u, rc0.col() + 1u) = back;
				grid(rc0.row()     , rc0.col() - 1u) = back;
				grid(rc0.row()     , rc0.col()     ) = fore;
				grid(rc0.row()     , rc0.col() + 1u) = back;
				grid(rc0.row() + 1u, rc0.col() - 1u) = back;
				grid(rc0.row() + 1u, rc0.col()     ) = back;
				grid(rc0.row() + 1u, rc0.col() + 1u) = back;
			}
		}

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
		( ras::Grid<Type> const & grid
		, img::Spot const & at
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

} // [ras]

} // [quadloco]

