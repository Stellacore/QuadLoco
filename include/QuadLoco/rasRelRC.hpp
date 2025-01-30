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
 * \brief Declarations for quadloco::ras::RelRC namespace
 *
 */


#include "QuadLoco/rasRowCol.hpp"

#include <iostream>
#include <limits>
#include <sstream>
#include <string>


namespace quadloco
{

namespace ras
{

	//! \brief Relative row/col position (wrt an 'active' filter center pixel)
	struct RelRC
	{
		// offsets relative to filter center
		int theRelRow{ std::numeric_limits<int>::max() };
		int theRelCol{ std::numeric_limits<int>::max() };


		//! True if this instance has valid data
		inline
		bool
		isValid
			() const
		{
			return
				(  (theRelRow < std::numeric_limits<int>::max())
				&& (theRelCol < std::numeric_limits<int>::max())
				);
		}

		//! Compute sum of indices with casting/test to avoid negative offsets
		inline
		std::size_t
		unsignedIndexFor
			( std::size_t const & srcCenterNdx
			, int const & ndxDelta
			) const
		{
			int const iNdx{ static_cast<int>(srcCenterNdx) + ndxDelta };
			if (iNdx < 0)
			{
				std::cerr << "Failure of iNdx positive check\n";
				std::cerr << "  srcCenterNdx: " << srcCenterNdx << '\n';
				std::cerr << "      ndxDelta: " << ndxDelta << '\n';
				exit(1);
			}
			return static_cast<std::size_t>(iNdx);
		}

		//! Absolute source grid (row,col) given filter center (row0,col0)
		inline
		ras::RowCol
		srcRowCol
			( std::size_t const & srcRow0
			, std::size_t const & srcCol0
			) const
		{
			return ras::RowCol
				{ unsignedIndexFor(srcRow0, theRelRow)
				, unsignedIndexFor(srcCol0, theRelCol)
				};
		}

		//! Absolute source grid (row,col) given filter center (row0,col0)
		inline
		ras::RowCol
		srcRowCol
			( ras::RowCol const & srcRowCol0
			) const
		{
			return srcRowCol(srcRowCol0.row(), srcRowCol0.col());
		}

		//! True if this and other instances have identical relative indices.
		inline
		bool
		operator==
			( RelRC const & other
			) const
		{
			return
				(  (theRelRow == other.theRelRow)
				&& (theRelCol == other.theRelCol)
				);
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
				oss << title << ' ';
			}
			oss
				<< "relRC:"
				<< ' ' << std::setw(4u) << theRelRow
				<< ' ' << std::setw(4u) << theRelCol
				;

			return oss.str();
		}



	}; // RelRC



} // [ras]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::ras::RelRC const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::ras::RelRC const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

