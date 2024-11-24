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
 * \brief Declarations for quadloco::img::Sampler
 *
 */


#include "rasGrid.hpp"
#include "rasRowCol.hpp"
#include "imgGrad.hpp"

#include <limits>


namespace quadloco
{

namespace img
{

	//! Extract derived data from grid
	class Sampler
	{
		ras::Grid<float> const * const thePtGrid{ nullptr };
		std::size_t const theStepHalf{ 1u };
		std::size_t const theStepFull{ 2u };
		float const theScale{ std::numeric_limits<float>::quiet_NaN() };

	public:

		//! Attach to *EXTERNALLY MANAGED* grid
		inline
		explicit
		Sampler
			( ras::Grid<float> const * const ptGrid = nullptr
			, std::size_t const stepHalf = 1u
			)
			: thePtGrid{ ptGrid }
			, theStepHalf{ stepHalf }
			, theStepFull{ 2u * theStepHalf }
			, theScale{ 1.f / (float)theStepFull }
		{ }

		//! Gradient computed at individual pixel - NO bounds checking
		inline
		img::Grad
		pixGradAt
			( ras::RowCol const & rowcol
				//!< Assumed to be inside extents of attached grid
			) const
		{
			img::Grad grad;
			if (thePtGrid)
			{
				//! Determine start and end indices
				std::size_t const rowNdxBeg
					{ theStepHalf };
				std::size_t const rowNdxEnd
					{ rowNdxBeg + thePtGrid->hwSize().high() - theStepFull };
				std::size_t const colNdxBeg
					{ theStepHalf };
				std::size_t const colNdxEnd
					{ colNdxBeg + thePtGrid->hwSize().wide() - theStepFull };

				ras::Grid<float> const & inGrid = *thePtGrid;
				float const & scl = theScale;

				std::size_t const & row = rowcol.row();
				std::size_t const & col = rowcol.col();

				bool const rowIn{ (! (row < rowNdxBeg)) && (row < rowNdxEnd) };
				bool const colIn{ (! (col < colNdxBeg)) && (col < colNdxEnd) };

				if (rowIn && colIn)
				{
					std::size_t const rowM1{ row - theStepHalf };
					std::size_t const rowP1{ row + theStepHalf };

					std::size_t const colM1{ col - theStepHalf };
					std::size_t const colP1{ col + theStepHalf };

					float const rowGrad
						{ scl * (inGrid(rowP1, col) - inGrid(rowM1, col)) };
					float const colGrad
						{ scl * (inGrid(row, colP1) - inGrid(row, colM1)) };

					grad = Grad{ rowGrad, colGrad };
				}
			}
			return grad;
		}

	}; // Sampler


} // [img]

} // [quadloco]

