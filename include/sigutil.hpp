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
	//! Convert grid elements to float
	inline
	ras::Grid<float>
	toFloat
		( ras::Grid<uint8_t> const & uGrid
			//!< Input source grid
		, int const & treatAsNull = -1
			//!< If value in range [0,255], then corresponding output is null
		)
	{
		ras::Grid<float> fGrid(uGrid.hwSize());
		ras::Grid<uint8_t>::const_iterator inIter{ uGrid.cbegin() };
		ras::Grid<float>::iterator outIter{ fGrid.begin() };
		while (uGrid.cend() != inIter)
		{
			uint8_t const & inVal = *inIter;
			float rtVal{ static_cast<float>(inVal) };
			bool const checkForNull{ ! (treatAsNull < 0) };
			if (checkForNull)
			{
				if (treatAsNull == inVal)
				{
					rtVal = std::numeric_limits<float>::quiet_NaN();
				}
			}
			*outIter = rtVal;
			inIter++;
			outIter++;
		}
		return fGrid;
	}

	//! A Larger grid produced with (nearest neighbor) up sampling.
	template <typename PixType>
	inline
	ras::Grid<PixType>
	toLarger
		( ras::Grid<PixType> const & inGrid
		, std::size_t const & upFactor
		)
	{
		ras::Grid<PixType> upGrid
			( upFactor * inGrid.high()
			, upFactor * inGrid.wide()
			);
		for (std::size_t inRow{0u} ; inRow < inGrid.high() ; ++inRow)
		{
			std::size_t const outRow0{ upFactor * inRow };
			for (std::size_t inCol{0u} ; inCol < inGrid.wide() ; ++inCol)
			{
				std::size_t const outCol0{ upFactor * inCol };
				for (std::size_t wRow{0u} ; wRow < upFactor ; ++wRow)
				{
					std::size_t const outRow{ outRow0 + wRow };
					for (std::size_t wCol{0u} ; wCol < upFactor ; ++wCol)
					{
						std::size_t const outCol{ outCol0 + wCol };
						upGrid(outRow, outCol) = inGrid(inRow, inCol);
					}
				}
			}
		}
		return upGrid;
	}

	//! Return grid containing result of applying function to each input cell
	template <typename Func>
	inline
	ras::Grid<float>
	resultGridFor
		( ras::Grid<img::Grad> const & gradGrid
		, Func const & funcOfGrad
		)
	{
		ras::Grid<float> outGrid(gradGrid.hwSize());
		std::transform
			( gradGrid.cbegin(), gradGrid.cend()
			, outGrid.begin()
			, funcOfGrad
			);
		return outGrid;
	}

	//! Convert gradients to magnitude
	inline
	ras::Grid<float>
	rowCompGridFor
		( ras::Grid<img::Grad> const & gradGrid
		)
	{
		auto const rowCompFunc
			{ [] (img::Grad const & grad) { return grad.drow(); } };
		return resultGridFor(gradGrid, rowCompFunc);
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
		return resultGridFor(gradGrid, colCompFunc);
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
		return resultGridFor(gradGrid, magFunc);
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
		return resultGridFor(gradGrid, angFunc);
	}


	//! Draw a (bright on black) radiometric mark at spot
	template <typename PixType>
	inline
	void
	drawSpot
		( ras::Grid<PixType> * const & ptGrid
		, img::Spot const & spot
		)
	{
		if (ptGrid && (2u < ptGrid->high()) && (2u < ptGrid->wide()))
		{
			ras::Grid<PixType> & grid = *ptGrid;

			// find min max
			using InIt = ras::Grid<PixType>::const_iterator;
			std::pair<InIt, InIt> const iterMM
				{ ops::grid::minmax_valid(grid.cbegin(), grid.cend()) };
			PixType const min{ *(iterMM.first) };
			PixType const max{ *(iterMM.second) };

			// boundary clipping
			img::Area const clipArea
				{ img::Span{ 1., (double)(ptGrid->high() - 1u) }
				, img::Span{ 1., (double)(ptGrid->wide() - 1u) }
				};

			PixType const & back = min;
			PixType const & fore = max;
			ras::RowCol const rc0{ cast::rasRowCol(spot) };
			if (clipArea.contains(cast::imgSpot(rc0)))
			{
				grid(rc0.row() - 1u, rc0.col() - 1u) = back;
				grid(rc0.row() - 1u, rc0.col()     ) = back;
				grid(rc0.row() - 1u, rc0.col() + 1u) = back;
				grid(rc0.row()     , rc0.col() - 1u) = back;
				grid(rc0.row()     , rc0.col()     ) = fore;
				grid(rc0.row()     , rc0.col() + 1u) = back;
				grid(rc0.row() + 1u, rc0.col() - 1u) = back;
				grid(rc0.row() + 1u, rc0.col()     ) = back;
				grid(rc0.row() + 1u, rc0.col() + 1u) = back;
			}
		}

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
		std::fill(eiGrid.begin(), eiGrid.end(), 0.f);
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

