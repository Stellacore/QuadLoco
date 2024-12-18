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
 * \brief Declarations for quadloco::ras::kernel namespace
 *
 */


#include "rasGrid.hpp"

#include <algorithm>
#include <cmath>


namespace quadloco
{

namespace ras
{

/*! \brief Digital filtering kernels (e.g. ref ops::grid::filter())
 */
namespace kernel
{
	//! Filter size that is odd number with symmetric halfSize cells each side
	inline
	ras::SizeHW
	hwSizeForHalfSizes
		( std::size_t const & halfHigh
		, std::size_t const & halfWide
		)
	{
		return ras::SizeHW
			{ 1u + 2u * halfHigh
			, 1u + 2u * halfWide
			};
	}

	//! A filter containing a (unit integral) Gaussian response
	template <typename Type>
	inline
	ras::Grid<Type>
	gauss
		( std::size_t const & halfSize
		, double const & sigma
		)
	{
		// minimum filter size is 1,1 (for halfSize of zero)
		ras::Grid<Type> filter(hwSizeForHalfSizes(halfSize, halfSize));

		// set initial (unweighted) filter values
		double const xy0{ static_cast<double>(halfSize) };
		double sum{ 0. };
		for (std::size_t row{0u} ; row < filter.high() ; ++row)
		{
			double const xx{ static_cast<double>(row) - xy0 };
			for (std::size_t col{0u} ; col < filter.wide() ; ++col)
			{
				double const yy{ static_cast<double>(col) - xy0 };

				double const radSq{ xx*xx + yy*yy };
				double const value{ std::exp(-radSq) };
				sum += value;
				filter(row, col) = static_cast<Type>(value);
			}
		}

		// normalize filter to unit integral weight
		if (0. < sum)
		{
			double const scl{ 1. / sum };
			std::transform
				( filter.cbegin(), filter.cend()
				, filter.begin()
				, [&scl] (Type const & val)
					{
						return static_cast<Type>
							(scl * static_cast<double>(val));
					}
				);
		}

		return filter;
	}

} // [kernel]


} // [ras]

} // [quadloco]

