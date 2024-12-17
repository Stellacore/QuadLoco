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
	inline
	ras::Grid<float>
	toSmooth
		( ras::Grid<float> const & inGrid
//		, std::size_t const & wHalf
		)
	{
		ras::Grid<float> outGrid(inGrid.hwSize());
		constexpr float nan{ std::numeric_limits<double>::quiet_NaN() };
		std::fill(outGrid.begin(), outGrid.end(), nan);

		constexpr int wHalf{ 1 };
		constexpr int wSize{ 2*wHalf + 1 };
		ras::Grid<float> wgts(wSize, wSize);
		wgts(0,0) = .5f;
		wgts(0,1) = .8f;
		wgts(0,2) = .5f;
		wgts(1,0) = .8f;
		wgts(1,1) = 1.f;
		wgts(1,2) = .8f;
		wgts(2,0) = .5f;
		wgts(2,1) = .8f;
		wgts(2,2) = .5f;
		float const sumWgts{ 4.f*.5f + 4.f*.8f + .1f };

		int const inRowEnd{ static_cast<int>(inGrid.high()) - wHalf };
		int const inColEnd{ static_cast<int>(inGrid.wide()) - wHalf };
		for (int inRow{wHalf} ; inRow < inRowEnd ; ++inRow)
		{
			int const inRow0{ inRow - wHalf };
			for (int inCol{wHalf} ; inCol < inColEnd ; ++inCol)
			{
				int const inCol0{ inCol - wHalf };

				// window processing
				float sumVals{ 0.f };
				for (int wRow{0} ; wRow < 2*wHalf ; ++wRow)
				{
					int const inRow{ inRow0 + wRow };
					for (int wCol{0} ; wCol < 2*wHalf ; ++wCol)
					{
						int const inCol{ inCol0 + wCol };
						sumVals += wgts(wRow, wCol) * inGrid(inRow, inCol);
					}
				}
				outGrid(inRow, inCol) = sumVals / sumWgts;

			}
		}
		return outGrid;
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

	//! Convert gradients to magnitude
	inline
	ras::Grid<float>
	magnitudeGridFor
		( ras::Grid<img::Grad> const & gradGrid
		)
	{
		ras::Grid<float> magGrid(gradGrid.hwSize());
		std::fill
			( magGrid.begin(), magGrid.end()
			, std::numeric_limits<float>::quiet_NaN()
			);

		ras::Grid<img::Grad>::const_iterator inIter{ gradGrid.cbegin() };
		ras::Grid<float>::iterator outIter{ magGrid.begin() };
		while (gradGrid.cend() != inIter)
		{
			*outIter++ = magnitude(*inIter++);
		}
		return magGrid;
	}

	//! Convert gradients to angle of grdient
	inline
	ras::Grid<float>
	angleGridFor
		( ras::Grid<img::Grad> const & gradGrid
		)
	{
		ras::Grid<float> angGrid(gradGrid.hwSize());
		std::fill
			( angGrid.begin(), angGrid.end()
			, std::numeric_limits<float>::quiet_NaN()
			);

		ras::Grid<img::Grad>::const_iterator inIter{ gradGrid.cbegin() };
		ras::Grid<float>::iterator outIter{ angGrid.begin() };
		while (gradGrid.cend() != inIter)
		{
			double angle{ std::numeric_limits<double>::quiet_NaN() };
			img::Grad const grad{ *inIter++ };
			if (isValid(grad))
			{
				img::Vector<double> const dir{ direction(grad) };
				angle = ang::atan2(dir[1], dir[0]);
			}
			*outIter++ = angle;
		}
		return angGrid;
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
				{ std::minmax_element(grid.cbegin(), grid.cend()) };
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

