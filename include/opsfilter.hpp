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
 * \brief Declarations for quadloco::ops::filter namespace
 *
 */


#include "rasGrid.hpp"

#include <Engabra>

#include <cstddef>


namespace quadloco
{

namespace ops
{

namespace filter
{
	//! Square val (multiplication by self)
	template <typename Type>
	inline
	Type
	sq
		( Type const & val
		)
	{
		return val*val;
	}

	// [DoxyExampleBoxFunc]

	/*! \brief Standard digital filter accumulation functor (for data of 'Type')
	 *
	 * Type is assumed to be real-like (i.e. are members of an algebraic
	 * field) - e.g. 'float', 'double', 'complex', etc.
	 */
	template <typename OutType, typename SrcType>
	struct WeightedSum
	{
		//! Filter weights (arbitrarily set by consumer)
		ras::Grid<OutType> const * const ptFilter;

		//! Filter response: updated by consider(), reported by operator()
		OutType theSum{ static_cast<OutType>(0) };

		//! Zero running sum (e.g. call every new window position)
		inline
		void
		reset
			( SrcType const & // srcValueAtCenter
				// source cell value not needed here for simple filters
			)
		{
			theSum = static_cast<SrcType>(0);
		}

		//! Weight srcValue by filter(wRow,wCol) and add into sum
		inline
		void
		consider
			( SrcType const & srcCellValue
			, std::size_t const & wRow
			, std::size_t const & wCol
			)
		{
			if (engabra::g3::isValid(srcCellValue))
			{
				OutType const & wgt = (*ptFilter)(wRow, wCol);
				theSum += wgt * static_cast<OutType>(srcCellValue);
			}
		}

		//! Filter-weighted sum of values consider()'ed
		inline
		OutType const &
		operator()
			() const
		{
			return theSum;
		}

	};  // WeightedSum

	// [DoxyExampleBoxFunc]



	/*! \brief Sum-squared-difference from center
	 *
	 * Type is assumed to be real-like (i.e. are members of an algebraic
	 * field) - e.g. 'float', 'double', 'complex', etc.
	 */
	template <typename Type>
	struct SumSquareDiff
	{
		//! Reference value (e.g. window center cell value)
		Type theRefValue{ static_cast<Type>(0) };

		//! Filter response: updated by consider(), reported by operator()
		Type theSumSq{ static_cast<Type>(0) };

		//! Zero running sum (e.g. call every new window position)
		inline
		void
		reset
			( Type const & srcValueAtCenter
				//!< Accumulate sum-square-diffs relative to this value
			)
		{
			theSumSq = static_cast<Type>(0);
			theRefValue = srcValueAtCenter;
		}

		//! Weight srcValue by filter(wRow,wCol) and add into sum
		inline
		void
		consider
			( Type const & srcCellValue
			, std::size_t const & // wRow
			, std::size_t const & // wCol
			)
		{
			theSumSq += sq(srcCellValue - theRefValue);
		}

		//! Filter-weighted sum of values consider()'ed
		inline
		Type const &
		operator()
			() const
		{
			return theSumSq;
		}

	};  // SumSquareDiff


} // [filter]

} // [ops]

} // [quadloco]

