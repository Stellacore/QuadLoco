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
 * \brief Declarations functions and utility for assessing "quadness"
 *
 */


#include "datGrid.hpp"
#include "imgQuadTarget.hpp"


namespace quadloco
{

namespace prb
{

	//! A (pseudo)probabilty values in pixGrid conform with image quad signal
	template <typename Type>
	inline
	double
	isQuadlike
		( dat::Grid<Type> const & pixGrid
		, img::QuadTarget const & imgQuad
		)
	{
		return .0; // TODO
	}


} // [prb]

} // [quadloco]

