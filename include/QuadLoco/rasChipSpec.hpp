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
 * \brief Declarations for quadloco::ras::ChipSpec
 *
 */


#include "QuadLoco/imgSpot.hpp"
#include "QuadLoco/rasGrid.hpp"
#include "QuadLoco/rasRowCol.hpp"
#include "QuadLoco/rasSizeHW.hpp"


namespace quadloco
{

namespace ras
{

	/*! \brief Maps locations between full size raster and a working sub area.
	 *
	 * This class provides a geometric remapping between its own row/col
	 * coordinates and some (presumed) external "fullsize" raster which
	 * is larger than the chip.
	 *
	 */
	struct ChipSpec
	{
		//! \brief Chip upper left corner is at this location in source image.
		ras::RowCol theSrcOrigRC;

		//! The height/width of this chip.
		ras::SizeHW theChipSizeHW;


		//! \brief ChipSpec centered on rcCenter with size (2u*half{H,W} + 1u)
		inline
		static
		ChipSpec
		centeredOn
			( ras::RowCol const & rcCenter
			, std::size_t const & halfHigh
			, std::size_t const & halfWide
			)
		{
			ChipSpec spec{};
			if ( (! (rcCenter.row() < halfHigh))
			  && (! (rcCenter.col() < halfWide))
			   )
			{
				ras::RowCol const rcOrig
					{ (rcCenter.row() - halfHigh)
					, (rcCenter.col() - halfWide)
					};
				ras::SizeHW const hwSize{ halfHigh, halfWide };
				spec = ChipSpec{ rcOrig, hwSize };
			}
			return spec;
		}

		//! \brief True if this instance contains non trivial values
		inline
		bool
		isValid
			() const
		{
			return theChipSizeHW.isValid();
		}

		//! \brief The row/column in source image for this chip's UL corner
		inline
		ras::RowCol const &
		srcOrigRC
			() const
		{
			return theSrcOrigRC;
		}

		//! \brief Source spot location at center of chip
		inline
		img::Spot
		srcCenterSpot
			() const
		{
			return img::Spot
				{ static_cast<double>(srcOrigRC().row())
					+ (.5 * static_cast<double>(high()))
				, static_cast<double>(srcOrigRC().col())
					+ (.5 * static_cast<double>(wide()))
				};
		}

		//! \brief Size of this chip - convenience for theChipSizeHW
		inline
		ras::SizeHW const &
		hwSize
			() const
		{
			return theChipSizeHW;
		}

		//! \brief Number rows in chip - convenience for theChipSizeHW.high()
		inline
		std::size_t
		high
			() const
		{
			return theChipSizeHW.high();
		}

		//! \brief Number cols in chip - convenience for theChipSizeHW.wide()
		inline
		std::size_t
		wide
			() const
		{
			return theChipSizeHW.wide();
		}

		//! \brief Row within (implicit) full raster where this chip starts
		inline
		std::size_t const &
		srcRowBeg
			() const
		{
			return theSrcOrigRC.row();
		}

		//! \brief Column within (implicit) full raster where this chip starts
		inline
		std::size_t const &
		srcColBeg
			() const
		{
			return theSrcOrigRC.col();
		}

		//! \brief Row within (implicit) full raster where this chip ends
		inline
		std::size_t
		srcRowEnd
			() const
		{
			return (srcRowBeg() + high());
		}

		//! \brief Column within (implicit) full raster where this chip ends
		inline
		std::size_t
		srcColEnd
			() const
		{
			return (srcColBeg() + wide());
		}

		//! \brief True if this chip region entirely fits inside fullSizeHW.
		inline
		bool
		fitsInto
			( ras::SizeHW const & fullSizeHW
			) const
		{
			// Note that high/wide are one more than max row/col index
			// However, since using unsigned sizes, do NOT try to
			// subtract the 1u value here, but instead ...
			std::size_t const chipRowEndInFull
				{ theSrcOrigRC.row() + theChipSizeHW.high() };
			std::size_t const chipColEndInFull
				{ theSrcOrigRC.col() + theChipSizeHW.wide() };
			// ... account for the 1u difference in test here
			bool const rowIsIn{ chipRowEndInFull < (fullSizeHW.high()+1u) };
			bool const colIsIn{ chipColEndInFull < (fullSizeHW.wide()+1u) };
			return (rowIsIn && colIsIn);
		}

		//! \brief Chip row/col expression for full image row/col (no checking)
		inline
		ras::RowCol
		rcChipForFullRC
			( ras::RowCol const & rcInFull
			) const
		{
			// rcInChip = (rcInFull - theSrcOrigRC)
			return ras::RowCol
				{ rcInFull.row() - theSrcOrigRC.row()
				, rcInFull.col() - theSrcOrigRC.col()
				};
		}

		//! \brief Spot location in source image corresponding with chip spot
		inline
		img::Spot
		chipSpotForFullSpot
			( img::Spot const & fullSpot
			) const
		{
			// chipSpot = (fullSpot - theSrcOrigRC)
			return img::Spot
				{ fullSpot.row() - static_cast<double>(theSrcOrigRC.row())
				, fullSpot.col() - static_cast<double>(theSrcOrigRC.col())
				};
		}

		//! \brief Full image row/col from chip row/col (no checking)
		inline
		ras::RowCol
		rcFullForChipRC
			( ras::RowCol const & rcInChip
			) const
		{
			// rcInChip = (rcInFull - theSrcOrigRC)
			// rcInChip + theSrcOrigRC = rcInFull
			// rcInFull = rcInChip + theSrcOrigRC
			return ras::RowCol
				{ rcInChip.row() + theSrcOrigRC.row()
				, rcInChip.col() + theSrcOrigRC.col()
				};
		}

		//! \brief Spot location in source image corresponding with chip spot
		inline
		img::Spot
		fullSpotForChipSpot
			( img::Spot const & chipSpot
			) const
		{
			// rcInFull = chipSpot + theSrcOrigRC
			return img::Spot
				{ chipSpot.row() + static_cast<double>(theSrcOrigRC.row())
				, chipSpot.col() + static_cast<double>(theSrcOrigRC.col())
				};
		}

		//! \brief The row/col location into (an assumed) full size image
		inline
		ras::RowCol
		fullRowColFor
			( std::size_t const & rowInChip
			, std::size_t const & colInChip
			) const
		{
			return fullRowColFor(ras::RowCol{ rowInChip, colInChip });
		}

		//! \brief The row/col location into (an assumed) full size image
		inline
		ras::RowCol
		fullRowColFor
			( ras::RowCol const & rcInChip
			) const
		{
			return rcFullForChipRC(rcInChip);
		}

		//! \brief Descriptive information about this instance.
		inline
		std::string
		infoString
			( std::string const & title = {}
			) const
		{
			std::ostringstream oss;
			if (!title.empty())
			{
				oss << title << ' ';
			}
			oss
				<< "origRC: " << theSrcOrigRC
				<< ' '
				<< "sizeHW: " << theChipSizeHW
				;
			return oss.str();
		}

	}; // ChipSpec


} // [ras]

} // [quadloco]


namespace
{
	//! \brief Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::ras::ChipSpec const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! \brief True if item is not null
	inline
	bool
	isValid
		( quadloco::ras::ChipSpec const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

