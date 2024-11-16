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
 * \brief Declarations for quadloco::dat::MapSizeArea
 *
 */


#include "datArea.hpp"
#include "datSizeHW.hpp"
#include "datSpan.hpp"
#include "datSpot.hpp"

#include <array>


namespace quadloco
{

namespace dat
{
	struct MapSizeArea
	{
		dat::Area theSize{};
		dat::Area theArea{};

		//! A mapping between "grid" space and "area" space
		inline
		explicit
		MapSizeArea
			( SizeHW const & hwGridSize
			, Span const & areaSpan0
			, Span const & areaSpan1
			)
			: theSize
				{ Span{ 0., (double)hwGridSize.high() }
				, Span{ 0., (double)hwGridSize.wide() }
				}
			, theArea
				{ areaSpan0
				, areaSpan1
				}
		{ }

		//! True if all data members are valid
		inline
		bool
		isValid
			() const
		{
			return
				(  theSize.isValid()
				&& theArea.isValid()
				);
		}

		//! Area spot associated with size spot location
		inline
		Spot
		areaSpotForGridSpot
			( Spot const & sizeSpot
			) const
		{
			Area::Dyad const fracSpot{ theSize.fractionDyadAtSpot(sizeSpot) };
			dat::Spot const areaSpot{ theArea.spotAtFractionDyad(fracSpot) };
			return areaSpot;
		}

		//! Grid spot location associated with area spot
		inline
		Spot
		gridSpotForAreaSpot
			( Spot const & areaSpot
			) const
		{
			Area::Dyad const fracSpot{ theArea.fractionDyadAtSpot(areaSpot) };
			dat::Spot const sizeSpot{ theSize.spotAtFractionDyad(fracSpot) };
			return sizeSpot;
		}

		//! Descriptive information about this instance.
		inline
		std::string
		infoString
			( std::string const & title = {}
			) const
		{
			std::ostringstream oss;
			if (! title.empty())
			{
				oss << title << '\n';
			}
			oss << "Grid: " << theSize << '\n';
			oss << "Area: " << theArea << '\n';
			return oss.str();
		}


	}; // MapSizeArea


} // [dat]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::dat::MapSizeArea const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::dat::MapSizeArea const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

