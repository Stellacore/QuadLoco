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
 * \brief Declarations for functions in quadloco::sig::util:: subnamespace
 *
 */

//#include "io.hpp"
//#include "opsgrid.hpp"


#include "cast.hpp"
#include "imgArea.hpp"
#include "opsgrid.hpp"
#include "rasgrid.hpp"
#include "rasGrid.hpp"
#include "sigEdgeInfo.hpp"

#include <algorithm>
#include <functional>
#include <vector>



namespace quadloco
{

namespace sig
{

/*! \brief Namespaced functions and utilities for signal processing
 */
namespace util
{
	//! Convert gradients to magnitude
	inline
	ras::Grid<float>
	rowCompGridFor
		( ras::Grid<img::Grad> const & gradGrid
		)
	{
		auto const rowCompFunc
			{ [] (img::Grad const & grad) { return grad.drow(); } };
		return ras::grid::resultGridFor(gradGrid, rowCompFunc);
	}

	//! Convert gradients to magnitude
	inline
	ras::Grid<float>
	colCompGridFor
		( ras::Grid<img::Grad> const & gradGrid
		)
	{
		auto const colCompFunc
			{ [] (img::Grad const & grad) { return grad.dcol(); } };
		return ras::grid::resultGridFor(gradGrid, colCompFunc);
	}

	//! Convert gradients to magnitude
	inline
	ras::Grid<float>
	magnitudeGridFor
		( ras::Grid<img::Grad> const & gradGrid
		)
	{
		auto const magFunc
			{ [] (img::Grad const & grad) { return magnitude(grad); } };
		return ras::grid::resultGridFor(gradGrid, magFunc);
	}

	//! Convert gradients to angle of grdient
	inline
	ras::Grid<float>
	angleGridFor
		( ras::Grid<img::Grad> const & gradGrid
		)
	{
		auto const angFunc
			{ [] (img::Grad const & grad)
				{ return ang::atan2(grad.dy(), grad.dx()); }
			};
		return ras::grid::resultGridFor(gradGrid, angFunc);
	}



	//! \brief Grid of values with cells set to func(edgeInfo).
	inline
	ras::Grid<float>
	edgeInfoGrid
		( ras::SizeHW const & hwSize
		, std::vector<sig::EdgeInfo> const & edgeInfos
		, std::function<float(sig::EdgeInfo const & ei)> const & func
		)
	{
		ras::Grid<float> eiGrid(hwSize);
		constexpr float nan{ std::numeric_limits<float>::quiet_NaN() };
		std::fill(eiGrid.begin(), eiGrid.end(), nan);
		for (sig::EdgeInfo const & edgeInfo : edgeInfos)
		{
			ras::RowCol const rowcol
				{ cast::rasRowCol(edgeInfo.edgel().start()) };
			eiGrid(rowcol) = func(edgeInfo);
		}
		return eiGrid;
	}


	//! \brief Grid of values with cells set to edgeInfo.consideredWeight()
	inline
	ras::Grid<float>
	edgeInfoWeightGrid
		( ras::SizeHW const & hwSize
		, std::vector<sig::EdgeInfo> const & edgeInfos
		)
	{
		return edgeInfoGrid
			( hwSize
			, edgeInfos
			, [] (sig::EdgeInfo const & ei)
				{ return (float)ei.consideredWeight(); }
			);
	}

	//! \brief Grid of values with cells set to edgeInfo.consideredAngle()
	inline
	ras::Grid<float>
	edgeInfoAngleGrid
		( ras::SizeHW const & hwSize
		, std::vector<sig::EdgeInfo> const & edgeInfos
		)
	{
		return edgeInfoGrid
			( hwSize
			, edgeInfos
			, [] (sig::EdgeInfo const & ei)
				{ return (float)ei.consideredAngle(); }
			);
	}


} // [util]

} // [sig]

} // [quadloco]

