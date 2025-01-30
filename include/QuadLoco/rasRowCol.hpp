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
 * \brief Declarations for ras::RowCol
 *
 */


#include <array>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>


namespace quadloco
{

namespace ras
{

	//! Discrete grid location in row,colum order.
	struct RowCol
	{
		//! Aggregate type row/col index data.
		std::array<std::size_t, 2u> theRowCol
			{ std::numeric_limits<std::size_t>::max()
			, std::numeric_limits<std::size_t>::max()
			};

		//! True if this instance contains non-default data
		inline
		bool
		isValid
			() const
		{
			return
				(  (std::numeric_limits<std::size_t>::max() != theRowCol[0])
				&& (std::numeric_limits<std::size_t>::max() != theRowCol[1])
				);
		}

		//! Convenience access to to theRowCol[0]
		inline
		std::size_t const &
		row
			() const
		{
			return theRowCol[0];
		}

		//! Convenience access to theRowCol[1]
		inline
		std::size_t const &
		col
			() const
		{
			return theRowCol[1];
		}

		//! True if this and other have exactly the same data
		inline
		bool
		operator==
			( RowCol const & other
			) const
		{
			return
				(  (other.row() == row())
				&& (other.col() == col())
				);
		}

		//! Descriptive information about this instance
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
			if (isValid())
			{
				oss
					<< std::setw(5u) << row()
					<< ' ' << std::setw(5u) << col()
					;
			}
			else
			{
				oss << " <null>";
			}
			return oss.str();
		}

	}; // RowCol


} // [ras]

} // [quadloco]


namespace
{
	//! Put item to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::ras::RowCol const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::ras::RowCol const & item
		)
	{
		return item.isValid();
	}


	//! True if the two items are identical (mostly for template compatibility)
	inline
	bool
	nearlyEquals
		( quadloco::ras::RowCol const & itemA
		, quadloco::ras::RowCol const & itemB
		, double const & tol = 0.
		)
	{
		double const rA{ static_cast<double>(itemA.row()) };
		double const rB{ static_cast<double>(itemB.row()) };
		double const cA{ static_cast<double>(itemA.col()) };
		double const cB{ static_cast<double>(itemB.col()) };
		return
			(  (! (tol < std::abs(rA - rB)))
			&& (! (tol < std::abs(cA - cB)))
			);
		/*
		return
			(  (itemA.row() == itemB.row())
			&& (itemA.col() == itemB.col())
			);
		*/
	}

} // [anon/global]

