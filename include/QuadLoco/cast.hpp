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
 * \brief Top level file for quadloco::cast namespace
 *
 */


#include "QuadLoco/imgSpot.hpp"
#include "QuadLoco/imgVector.hpp"
#include "QuadLoco/rasRowCol.hpp"

#include <cmath>



namespace quadloco
{

/*! \brief Casting operations code in namespace quadloco::cast
 */
namespace cast
{
	//
	// To ras::RowCol
	//

	//! Integral values that are the std::floor values of datSpot[0,1]
	template <typename Type>
	inline
	ras::RowCol
	rasRowCol
		( img::Vector<Type> const & vec2D
		)
	{
		return ras::RowCol
			{ static_cast<std::size_t>(std::floor(vec2D[0]))
			, static_cast<std::size_t>(std::floor(vec2D[1]))
			};
	}

	//! Integral values that are the std::floor values of rasSpot[0,1]
	inline
	ras::RowCol
	rasRowCol
		( img::Spot const & rasSpot
		)
	{
		return ras::RowCol
			{ static_cast<std::size_t>(std::floor(rasSpot[0]))
			, static_cast<std::size_t>(std::floor(rasSpot[1]))
			};
	}

	//
	// To img::Vector<Type>
	//

	//
	// To img::Spot
	//

	//! Cast row,col values to double and package for return
	inline
	img::Spot
	imgSpot
		( ras::RowCol const & rowcol
		)
	{
		return img::Spot
			{ static_cast<double>(rowcol.row())
			, static_cast<double>(rowcol.col())
			};
	}

	//! Cast to img::Spot from img::Vector
	inline
	img::Spot
	imgSpot
		( img::Vector<double> const & vec
		)
	{
		return img::Spot{ vec[0], vec[1] };
	}

} // [cast]

} // [quadloco]

