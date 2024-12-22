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


#include "imgVector.hpp"
#include "rasGrid.hpp"


/*! \file
 * \brief Declarations for quadloco::ops namespace matrix operations
 *
 */


// #include "TODO.hpp"


/*
namespace quadloco
{

namespace ops
{


} // [NM]

} // [quadloco]
*/


namespace
{
	//! 2D Matrix (pre)multiplication: result = mat2D * vec2D
	inline
	quadloco::img::Vector<double>
	operator*
		( quadloco::ras::Grid<double> const & mat2D
		, quadloco::img::Vector<double> const & vec2D
		)
	{
		return quadloco::img::Vector<double>
			{ mat2D(0, 0)*vec2D[0] + mat2D(0, 1)*vec2D[1]
			, mat2D(1, 0)*vec2D[0] + mat2D(1, 1)*vec2D[1]
			};
	}

	//! Trace of 2x2 matrix
	inline
	double
	trace
		( quadloco::ras::Grid<double> const & mat2D
		)
	{
		return (mat2D(0, 0) + mat2D(1, 1));
	}

	//! Determinant of 2x2 matrix
	inline
	double
	determinant
		( quadloco::ras::Grid<double> const & mat2D
		)
	{
		return (mat2D(0, 0) * mat2D(1, 1) - mat2D(1, 0) * mat2D(0, 1));
	}

} // [anon/global]


