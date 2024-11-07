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
 * \brief Definitions for dat::Grid<> raster data structure
 *
 */


#include "datRowCol.hpp"
#include "datSizeHW.hpp"

#include <string>

// implementation
#include <stdio.h>
#include <sstream>


namespace quadloco
{

namespace dat
{
	/*! \brief Holds typed data in layout with row/cols.

	Data values are stored in row major order.
	Iterations proceed as (0, 0), (0, 1), (0, 2) etc. (along rows)

	\par Example
	\snippet test/test_datGrid.cpp DoxyExample01
	*/
	template < typename Type >
	class Grid
	{
		std::size_t theHigh{ 0u }; //!< Height of buffer
		std::size_t theWide{ 0u }; //!< Width of buffer
		Type * theData{ nullptr }; //!< data buffer pointer

	public: // typedef

		typedef Type value_type;
		typedef Type * iterator;
		typedef Type const * const_iterator;
		typedef std::reverse_iterator<const_iterator> const_reverse_iterator;
		typedef std::reverse_iterator<iterator> reverse_iterator;

	public: // static methods

		//! A null (zero sized) grid
		static
		Grid
		null
			()
		{
			return {};
		}

		// disable copy construction
		Grid
			(Grid const & other) = delete;

		// disable copy assignment
		Grid &
		operator=
			(Grid const & rhs) = delete;

	public: // methods

		//! Construct empty
		Grid
			() = default;

		//! Construct with geometry (NOTE: uninitialized values)
		inline
		explicit
		Grid
			( SizeHW const & hwSize
			)
			: theHigh{ hwSize.high() }
			, theWide{ hwSize.wide() }
			, theData{ nullptr }
		{
			if (0 < theHigh && 0 < theWide)
			{
				theData = new Type[theHigh * theWide];
			}
		}

		//! Move constructor
		inline
		Grid
			( Grid<Type> && orig
			)
			: Grid()
		{
			std::swap(theHigh, orig.theHigh);
			std::swap(theWide, orig.theWide);
			std::swap(theData, orig.theData);
		}

		//! Move assignment
		inline
		Grid<Type> &
		operator=
			( Grid<Type> && rhs
			)
		{
			if (&rhs != this)
			{
				std::swap(theHigh, rhs.theHigh);
				std::swap(theWide, rhs.theWide);
				std::swap(theData, rhs.theData);
			}
			return *this;
		}

		//! standard destructor
		inline
		~Grid
			()
		{
			if (theData)
			{
				delete [] theData;
			}
		}

		//! check if is valid
		inline
		bool
		isValid
			() const
		{
			return theData != nullptr;
		}

		//! Dimensions of this image
		inline
		SizeHW
		hwSize
			() const
		{
			return SizeHW(theHigh, theWide);
		}

		//! Number of cells in each column of the grid
		inline
		std::size_t
		high
			() const
		{
			return theHigh;
		}

		//! Number of cells in each row of the grid
		inline
		std::size_t
		wide
			() const
		{
			return theWide;
		}

		//! Number of cells in grid
		inline
		std::size_t
		size
			() const
		{
			return hwSize().size();
		}

		//! bytes in buffer
		inline
		size_t
		byteSize
			() const
		{
			return hwSize().size() * sizeof(Type);
		}

		//! Returns reference to element
		inline
		Type const &
		operator()
			( size_t const & row
			, size_t const & col
			) const
		{
			return *(theData + row * theWide + col);
		}

		//! Returns pointer to row (non const)
		inline
		Type &
		operator()
			( size_t const & row
			, size_t const & col
			)
		{
			return *(theData + row * theWide + col);
		}

		//! Returns reference to element
		inline
		Type const &
		operator()
			( dat::RowCol const & rowcol
			) const
		{
			return operator()(rowcol.theRow, rowcol.theCol);
		}

		//! Returns pointer to row (non const)
		inline
		Type &
		operator()
			( dat::RowCol const & rowcol
			)
		{
			return operator()(rowcol.theRow, rowcol.theCol);
		}

		//! use as const_iterator
		inline
		const_iterator
		cbegin
			() const
		{
			return theData;
		}

		//! use as const_iterator
		inline
		const_iterator
		cend
			() const
		{
			return theData + hwSize().size();
		}

		//! use as iterator
		inline
		iterator
		begin
			()
		{
			return theData;
		}

		//! use as iterator
		inline
		iterator
		end
			()
		{
			return theData + hwSize().size();
		}

		//! use as const_reverse_iterator
		inline
		const_reverse_iterator
		crbegin
			() const
		{
			return const_reverse_iterator(end());
		}

		//! use as const_reverse_iterator
		inline
		const_reverse_iterator
		crend
			() const
		{
			return const_reverse_iterator(begin());
		}

		//! use as reverse_iterator
		inline
		reverse_iterator
		rbegin
			()
		{
			return reverse_iterator(end());
		}

		//! use as reverse_iterator
		inline
		reverse_iterator
		rend
			()
		{
			return reverse_iterator(begin());
		}

		//! const row access, use as const iterator
		inline
		const_iterator
		cbeginRow
			( size_t const & row
			) const
		{
			return theData + (row * theWide);
		}

		//! const row access, use as const iterator
		inline
		const_iterator
		cendRow
			( size_t const & row
			) const
		{
			return beginRow(row + 1);
		}

		//! row access, use as iterator
		inline
		iterator
		beginRow
			( size_t const & row
			)
		{
			return theData + (row * theWide);
		}

		//! row access, use as iterator
		inline
		iterator
		endRow
			( size_t const & row
			)
		{
			return beginRow(row + 1);
		}

		//! iterator at given row, ocl
		inline
		const_iterator
		citerAt
			( size_t const & row
			, size_t const & col
			) const
		{
			return theData + (row * theWide + col);
		}

		//! iterator at given row, ocl
		inline
		iterator
		iterAt
			( size_t const & row
			, size_t const & col
			)
		{
			return theData + (row * theWide + col);
		}

		//! row/col from iterator
		inline
		dat::RowCol
		rowColFor
			( const_iterator const & iter
			) const
		{
			size_t const dist(std::distance(begin(), iter));
			return dat::RowCol
				{ dist / hwSize().wide()
				, dist % hwSize().wide()
				};
		}

		/*! \brief Descriptive information about this instance.
		 *
		 * Format is:
		 * \arg title "High,Wide" high wide "Cells,Bytes" ncells nbytes
		 */
		inline
		std::string
		infoString
			( std::string const & title=std::string()
			) const
		{
			std::ostringstream os;
			if (!title.empty())
			{
				os << title << " ";
			}
			SizeHW const hw{ hwSize() };
			os << "High,Wide:"
				<< ' ' << std::setw(5) << hw.high()
				<< ' ' << std::setw(5) << hw.wide()
				<< "  Cells,Bytes:"
				<< ' ' << std::setw(5) << size()
				<< ' ' << std::setw(5) << byteSize()
				;
			return os.str();
		}

		/*! \brief The content values of the this instance
		 *
		 * Format is:
		 * \arg infoString() as first line
		 * \arg each line is a row of values formatted with cfmt
		 */
		inline
		std::string
		infoStringContents
			( std::string const & title
				//!< Heading to print (along with size info)
			, std::string const & cfmt
				//!< A 'printf' style string to format each cell value
			) const
		{
			std::ostringstream oss;
			oss << infoString(title);
			if (isValid())
			{
				const_iterator iter{ cbegin() };
				for (size_t row(0); row < theHigh; ++row)
				{
					oss << '\n';
					for (size_t col(0); col < theWide; ++col)
					{
						constexpr std::size_t bufSize{ 1024u };
						char buf[bufSize+1u]; // hopefully large enough
						std::snprintf(buf, bufSize, cfmt.c_str(), *iter++);
						oss << ' ' << buf;
					}
				}
			}
			else
			{
				oss << " <null>";
			}
			return oss.str();
		}

/*
		//! exactly equals operator
		inline
		bool
		operator==
			( Grid const & rhs
			) const;

		//! Cast to different element type
		template <typename OutType>
		inline
		dat::Grid<OutType>
		castGrid
			() const;

		//! Number of valid column entires in specified row
		inline
		size_t
		validCountInRow
			( size_t const & rowNdx
			) const;

		//! Number of valid rows entires in specified column
		inline
		size_t
		validCountInCol
			( size_t const & colNdx
			) const;
*/

/*
		//! destructive resize
		inline
		void
		resize
			( size_t const & high
			, size_t const & wide
			);
*/

	};


} // [dat]

} // [quadloco]


namespace
{
	//! Put obj.infoString() to stream
	template < typename Type >
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::dat::Grid<Type> const & obj
		)
	{
		ostrm << obj.infoString();
		return ostrm;
	}

} // [anon/global]

