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
 * \brief Declarations for quadloco::img::ChipSpec
 *
 */


#include "rasGrid.hpp"
#include "rasRowCol.hpp"
#include "rasSizeHW.hpp"


namespace quadloco
{

namespace img
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
		ras::RowCol const theOrigRC;
		ras::SizeHW const theSizeHW;


		//! True if this instance contains non trivial values
		inline
		bool
		isValid
			() const
		{
			return theSizeHW.isValid();
		}

		//! Size of this chip - convenience for theSizeHW
		inline
		ras::SizeHW const &
		hwSize
			() const
		{
			return theSizeHW;
		}

		//! Number rows in this chip - convenience for theSizeHW.high()
		inline
		std::size_t
		high
			() const
		{
			return theSizeHW.high();
		}

		//! Number columns in this chip - convenience for theSizeHW.wide()
		inline
		std::size_t
		wide
			() const
		{
			return theSizeHW.wide();
		}

		//! Row within (implicit) full raster data where this chip starts
		inline
		std::size_t const &
		row0
			() const
		{
			return theOrigRC.row();
		}

		//! Column within (implicit) full raster data where this chip starts
		inline
		std::size_t const &
		col0
			() const
		{
			return theOrigRC.col();
		}

		//! True if this chip region entirely fits inside fullSizeHW.
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
				{ theOrigRC.row() + theSizeHW.high() };
			std::size_t const chipColEndInFull
				{ theOrigRC.col() + theSizeHW.wide() };
			// ... account for the 1u difference in test here
			bool const rowIsIn{ chipRowEndInFull < (fullSizeHW.high()+1u) };
			bool const colIsIn{ chipColEndInFull < (fullSizeHW.wide()+1u) };
			return (rowIsIn && colIsIn);
		}

		//! Chip row/col expression given full image row/col (no checking)
		inline
		ras::RowCol
		rcChipForFullRC
			( ras::RowCol const & rcInFull
			) const
		{
			// rcInChip = (rcInFull - theOrigRC)
			return ras::RowCol
				{ rcInFull.row() - theOrigRC.row()
				, rcInFull.col() - theOrigRC.col()
				};
		}

		//! Full image row/col expression from chip row/col (no checking)
		inline
		ras::RowCol
		rcFullForChipRC
			( ras::RowCol const & rcInChip
			) const
		{
			// rcInChip = (rcInFull - theOrigRC)
			// rcInChip + theOrigRC = rcInFull
			// rcInFull = rcInChip + theOrigRC
			return ras::RowCol
				{ rcInChip.row() + theOrigRC.row()
				, rcInChip.col() + theOrigRC.col()
				};
		}

		//! The row/col location into (an assumed) full size image
		inline
		ras::RowCol
		fullRowColFor
			( std::size_t const & rowInChip
			, std::size_t const & colInChip
			) const
		{
			return fullRowColFor(ras::RowCol{ rowInChip, colInChip });
		}

		//! The row/col location into (an assumed) full size image
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
				<< "origRC: " << theOrigRC
				<< ' '
				<< "sizeHW: " << theSizeHW
				;
			return oss.str();
		}

	}; // ChipSpec


} // [img]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::img::ChipSpec const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::img::ChipSpec const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

