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
 * \brief TODO
 *
 */


#include "datVec2D.hpp"
#include "pixGrad.hpp"


namespace quadloco
{

namespace fnd
{

	//! Direction perpendicular to gradient direction in right hand sense
	inline
	dat::Vec2D<double>
	lineDirFromEdgeDir
		( pix::Grad const & grad
		)
	{
		return dat::Vec2D<double>
			{ -(double)grad[1]
			,  (double)grad[0]
			};
	}

	//! Direction perpendicular to gradient direction in right hand sense
	template <typename Type>
	inline
	dat::Vec2D<Type>
	lineDirFromEdgeDir
		( dat::Vec2D<Type> const & edgeGradient
		)
	{
		return dat::Vec2D<Type>
			{ -edgeGradient[1]
			,  edgeGradient[0]
			};
	}





} // [fnd]

} // [quadloco]

