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
 * \brief Declarations for quadloco::ops namespace matrix operations
 *
 */


#include "imgVector.hpp"
#include "rasGrid.hpp"

#include <algorithm>


namespace quadloco
{

namespace ops
{

	//! Use ras::Grid as a matrix data container
	using Matrix = quadloco::ras::Grid<double>;

namespace matrix
{
	//
	// Matrix constructions
	//

	//! A (dim x dim) identity matrix
	inline
	Matrix
	identity
		( std::size_t const & dim
		)
	{
		Matrix mat(dim, dim);
		std::fill(mat.begin(), mat.end(), 0.);
		for (std::size_t nn{0u} ; nn < dim ; ++nn)
		{
			mat(nn, nn) = 1.;
		}
		return mat;
	}

	//! A diagonal matrix filled with elems along the diagonal
	inline
	Matrix
	diagonal
		( std::initializer_list<double> const & elems
		)
	{
		Matrix mat{ elems.size(), elems.size() };
		std::fill(mat.begin(), mat.end(), 0.);
		for (std::size_t nn{0u} ; nn < elems.size() ; ++nn)
		{
			mat(nn, nn) = elems.begin()[nn];
		}
		return mat;
	}

	//
	// Matrix functions
	//

	inline
	Matrix
	copyOf
		( Matrix const & srcMat
		)
	{
		Matrix outMat(srcMat.hwSize());
		std::copy
			( srcMat.cbegin(), srcMat.cend()
			, outMat.begin()
			);
		return outMat;
	}

} // [matrix]

} // [ops]

} // [quadloco]


namespace
{
	//
	// Global operators
	//

	//! 2D Matrix scalar (pre)multiplication
	inline
	quadloco::ras::Grid<double>
	operator*
		( double const & scale
		, quadloco::ras::Grid<double> const & matrix
		)
	{
		using namespace quadloco;
		ras::Grid<double> result(matrix.hwSize());
		ras::Grid<double>::const_iterator itIn{ matrix.cbegin() };
		ras::Grid<double>::iterator itOut{ result.begin() };
		while (result.end() != itOut)
		{
			*itOut++ = scale * (*itIn++);
		}
		return result;
	}

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


