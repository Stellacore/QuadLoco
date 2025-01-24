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
 * \brief Declarations for quadloco::mat namespace types and constructors
 *
 */


#include "rasGrid.hpp"

#include <algorithm>
#include <initializer_list>


namespace quadloco
{

namespace mat
{

	//! Use ras::Grid as a matrix data container
	using Matrix = quadloco::ras::Grid<double>;

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

} // [mat]

} // [quadloco]

