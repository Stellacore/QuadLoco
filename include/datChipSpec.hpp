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
 * \brief Declarations for quadloco::dat::ChipSpec
 *
 */


#include "datGrid.hpp"
#include "datRowCol.hpp"
#include "datSizeHW.hpp"


namespace quadloco
{

namespace dat
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
		RowCol const theOrigRC;
		SizeHW const theSizeHW;


		//! True if this instance contains non trivial values
		inline
		bool
		isValid
			() const
		{
			return theSizeHW.isValid();
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
			( SizeHW const & fullSizeHW
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
		RowCol
		rcChipForFullRC
			( RowCol const & rcInFull
			) const
		{
			// rcInChip = (rcInFull - theOrigRC)
			return RowCol
				{ rcInFull.row() - theOrigRC.row()
				, rcInFull.col() - theOrigRC.col()
				};
		}

		//! Full image row/col expression from chip row/col (no checking)
		inline
		RowCol
		rcFullForChipRC
			( RowCol const & rcInChip
			) const
		{
			// rcInChip = (rcInFull - theOrigRC)
			// rcInChip + theOrigRC = rcInFull
			// rcInFull = rcInChip + theOrigRC
			return RowCol
				{ rcInChip.row() + theOrigRC.row()
				, rcInChip.col() + theOrigRC.col()
				};
		}

		//! The row/col location into (an assumed) full size image
		inline
		RowCol
		fullRowColFor
			( std::size_t const & rowInChip
			, std::size_t const & colInChip
			) const
		{
			return fullRowColFor(RowCol{ rowInChip, colInChip });
		}

		//! The row/col location into (an assumed) full size image
		inline
		RowCol
		fullRowColFor
			( RowCol const & rcInChip
			) const
		{
			return rcFullForChipRC(rcInChip);
		}

		//! Populate ptChipData cells with values from fullData
		template <typename Type>
		inline
		bool
		fillChipFromFull
			( dat::Grid<Type> * const ptChipData
			, dat::Grid<Type> const & fullData
			) const
		{
			bool okay{ ptChipData && fitsInto(fullData.hwSize()) };
			if (okay)
			{
				std::size_t const highChip{ ptChipData->hwSize().high() };
				std::size_t const wideChip{ ptChipData->hwSize().wide() };
				for (std::size_t rChip{0u} ; rChip < highChip ; ++rChip)
				{
					for (std::size_t cChip{0u} ; cChip < wideChip ; ++cChip)
					{
						RowCol const rcChip{ rChip, cChip };
						RowCol const rcFull{ rcFullForChipRC(rcChip) };
						(*ptChipData)(rcChip) = fullData(rcFull);
					}
				}
			}
			return okay;
		}

		//! Populate ptFullData cells with values from chipData
		template <typename Type>
		inline
		bool
		fillFullFromChip
			( dat::Grid<Type> * const ptFullData
			, dat::Grid<Type> const & chipData
			) const
		{
			bool okay {ptFullData && fitsInto(ptFullData->hwSize()) };
			if (okay)
			{
				std::size_t const highChip{ chipData.hwSize().high() };
				std::size_t const wideChip{ chipData.hwSize().wide() };
				for (std::size_t rChip{0u} ; rChip < highChip ; ++rChip)
				{
					for (std::size_t cChip{0u} ; cChip < wideChip ; ++cChip)
					{
						RowCol const rcChip{ rChip, cChip };
						RowCol const rcFull{ rcFullForChipRC(rcChip) };
						(*ptFullData)(rcFull) = chipData(rcChip);
					}
				}
			}
			return okay;
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


} // [dat]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::dat::ChipSpec const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::dat::ChipSpec const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

