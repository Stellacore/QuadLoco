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
 * \brief Declarations for quadloco::xfm::MapSizeArea
 *
 */


#include "imgArea.hpp"
#include "rasSizeHW.hpp"
#include "sigSpan.hpp"
#include "imgSpot.hpp"

#include <array>


namespace quadloco
{

namespace dat
{
	//!< A 2D scaling transformation between "Size" and "Area" spaces.
	struct MapSizeArea
	{
		img::Area theSize{};
		img::Area theArea{};

		//! Interpretation at Area edges
		enum EdgeMode
			{ Clip //!< Return null for data outide of *theArea*
			, Wrap //!< Wrap data into *theArea*
			};

		EdgeMode theEdgeMode{};

		//! A mapping between "grid" space and "area" space
		inline
		explicit
		MapSizeArea
			( Area const & intoArea
			, Area const & fromArea
			, EdgeMode const & edgeMode = Clip
			)
			: theSize{ intoArea }
			, theArea{ fromArea }
			, theEdgeMode{ edgeMode }
		{ }

		//! A mapping between "grid" space and "area" space
		inline
		explicit
		MapSizeArea
			( SizeHW const & hwGridSize
			, Area const & fromArea
			, EdgeMode const & edgeMode = Clip
			)
			: MapSizeArea
				( img::Area
					{ Span{ 0., (double)hwGridSize.high() }
					, Span{ 0., (double)hwGridSize.wide() }
					}
				, fromArea
				, edgeMode
				)
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
			img::Spot areaSpot{};
			if (theSize.contains(sizeSpot))
			{
				Area::Dyad const fracDyad
					{ theSize.fractionDyadAtSpot(sizeSpot) };
				areaSpot = theArea.spotAtFractionDyad(fracDyad);
			}
			return areaSpot;
		}

		//! Grid spot location associated with area spot
		inline
		Spot
		gridSpotForAreaSpot
			( Spot const & areaSpot
			) const
		{
			img::Spot sizeSpot{};
			Area::Dyad const fracDyad{ theArea.fractionDyadAtSpot(areaSpot) };
			if (Clip == theEdgeMode)
			{
				if (theArea.contains(areaSpot))
				{
					sizeSpot = theSize.spotAtFractionDyad(fracDyad);
				}
			}
			else
			if (Wrap == theEdgeMode)
			{
				Area::Dyad const pFracDyad
					{ theArea.principalFractionDyad(fracDyad) };
				sizeSpot = theSize.spotAtFractionDyad(pFracDyad);
			}
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
			oss << "Grid: " << theSize
				<< '\n'
				<< "Area: " << theArea
				;
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
		, quadloco::xfm::MapSizeArea const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::xfm::MapSizeArea const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

