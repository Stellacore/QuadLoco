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
 * \brief Declarations for dat::RowCol
 *
 */


#include <iomanip>
#include <sstream>
#include <string>
#include <utility>


namespace quadloco
{

namespace dat
{

	//! Discrete grid location in row,colum order.
	struct RowCol
	{
		std::size_t const theRow{ 0u };
		std::size_t const theCol{ 0u };

		//! True if this and other have exactly the same data
		inline
		bool
		operator==
			( RowCol const & other
			) const
		{
			return
				(  (other.theRow == theRow)
				&& (other.theCol == theCol)
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
			oss
				<< std::setw(5u) << theRow
				<< ' ' << std::setw(5u) << theCol
				;
			return oss.str();
		}

	}; // RowCol


} // [dat]

} // [quadloco]


namespace
{
	//! Put obj.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::dat::RowCol const & obj
		)
	{
		ostrm << obj.infoString();
		return ostrm;
	}

} // [anon/global]

