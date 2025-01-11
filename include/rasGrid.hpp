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
 * \brief Definitions for ras::Grid<> raster data structure
 *
 */


#include "rasRowCol.hpp"
#include "rasSizeHW.hpp"

#include <functional>
#include <string>

// implementation
#include <stdio.h>
#include <sstream>


namespace quadloco
{

namespace ras
{
	/*! \brief Holds typed data in layout with row/cols.

	Data values are stored in row major order.
	Iterations proceed as (0, 0), (0, 1), (0, 2) etc. (along rows)

	\par Example
	\snippet test/test_rasGrid.cpp DoxyExample01
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

		// static methods

		//! DISABLE copy construction (use move sytax, or std::copy(...))
		Grid
			(Grid const & other) = delete;

		//! DISABLE assignment (use move sytax, or std::copy(...))
		Grid &
		operator=
			(Grid const & rhs) = delete;

		//! Examplicit null instance (zero size, no data). Same as Grid{};
		static
		Grid
		null
			()
		{
			return {};
		}

		//! Return a (deep) copy of original
		inline
		static
		Grid<Type>
		copyOf
			( Grid<Type> const & orig
			)
		{
			Grid<Type> copy{ orig.hwSize() };
			std::copy
				( orig.cbegin(), orig.cend()
				, copy.begin()
				);
			return copy;
		}
			

		// methods

		//! Construct a null instance (false == isValid()): (same Grid::null())
		Grid
			() = default;

		//! Construct geometry and allocate space (NOTE: uninitialized values)
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

		//! Convenience constructor taking individual high/wide args
		inline
		explicit
		Grid
			( std::size_t const & high
			, std::size_t const & wide
			)
			: Grid{ SizeHW{ high, wide } }
		{ }

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

		//! Standard destructor (deletes data store)
		inline
		~Grid
			()
		{
			if (theData)
			{
				delete [] theData;
				theHigh = 0u;
				theWide = 0u;
				theData = nullptr;
			}
		}

		//! True if this instance is not null (has data
		inline
		bool
		isValid
			() const
		{
			return (theData != nullptr);
		}

		//! Dimensions of this image
		inline
		SizeHW
		hwSize
			() const
		{
			return SizeHW(theHigh, theWide);
		}

		//! Number of cells in each column of the grid (== hwSize.high())
		inline
		std::size_t
		high
			() const
		{
			return theHigh;
		}

		//! Number of cells in each row of the grid (== hwSize.wide())
		inline
		std::size_t
		wide
			() const
		{
			return theWide;
		}

		//! Number of cells in grid (aka "element count")
		inline
		std::size_t
		size
			() const
		{
			return hwSize().size();
		}

		//! Number of bytes consumed by data = ((size()*sizeof(Type))
		inline
		size_t
		byteSize
			() const
		{
			return hwSize().size() * sizeof(Type);
		}

		//! Constant reference to specified element
		inline
		Type const &
		operator()
			( size_t const & row
			, size_t const & col
			) const
		{
			return *(theData + row * theWide + col);
		}

		//! Mutuable reference to specified element
		inline
		Type &
		operator()
			( size_t const & row
			, size_t const & col
			)
		{
			return *(theData + row * theWide + col);
		}

		//! Constant reference to specified element
		inline
		Type const &
		operator()
			( ras::RowCol const & rowcol
			) const
		{
			return operator()(rowcol.row(), rowcol.col());
		}

		//! Mutuable reference to specified element
		inline
		Type &
		operator()
			( ras::RowCol const & rowcol
			)
		{
			return operator()(rowcol.row(), rowcol.col());
		}

		//! Iterator to start of read only data
		inline
		const_iterator
		cbegin
			() const
		{
			return theData;
		}

		//! Iterator to end of read only data
		inline
		const_iterator
		cend
			() const
		{
			return theData + hwSize().size();
		}

		//! Iterator to start of mutable data
		inline
		iterator
		begin
			()
		{
			return theData;
		}

		//! Iterator to end of mutable data
		inline
		iterator
		end
			()
		{
			return theData + hwSize().size();
		}

		//! Reverse-direction iterator to start of read only data
		inline
		const_reverse_iterator
		crbegin
			() const
		{
			return const_reverse_iterator(end());
		}

		//! Reverse-direction iterator to end of read only data
		inline
		const_reverse_iterator
		crend
			() const
		{
			return const_reverse_iterator(begin());
		}

		//! Reverse-direction iterator to start of mutable data
		inline
		reverse_iterator
		rbegin
			()
		{
			return reverse_iterator(end());
		}

		//! Reverse-direction iterator to end of mutable data
		inline
		reverse_iterator
		rend
			()
		{
			return reverse_iterator(begin());
		}

		//! Iterator to start of *ROW* of read only data
		inline
		const_iterator
		cbeginRow
			( size_t const & row
			) const
		{
			return theData + (row * theWide);
		}

		//! Iterator to end of *ROW* of read only data
		inline
		const_iterator
		cendRow
			( size_t const & row
			) const
		{
			return beginRow(row + 1);
		}

		//! Iterator to start of *ROW* of mutable data
		inline
		iterator
		beginRow
			( size_t const & row
			)
		{
			return theData + (row * theWide);
		}

		//! Iterator to end of *ROW* of mutable data
		inline
		iterator
		endRow
			( size_t const & row
			)
		{
			return beginRow(row + 1);
		}

		//! Iterator to read-only data element at (row,col)
		inline
		const_iterator
		citerAt
			( size_t const & row
			, size_t const & col
			) const
		{
			return theData + (row * theWide + col);
		}

		//! Iterator to mutable data element at (row,col)
		inline
		iterator
		iterAt
			( size_t const & row
			, size_t const & col
			)
		{
			return theData + (row * theWide + col);
		}

		//! Row/Colum indices associated with iter value
		inline
		ras::RowCol
		rasRowColFor
			( const_iterator const & iter
			) const
		{
			size_t const dist(std::distance(cbegin(), iter));
			return ras::RowCol
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
			std::ostringstream oss;
			if (!title.empty())
			{
				oss << title << " ";
			}
			SizeHW const hw{ hwSize() };
			oss << "High,Wide:"
				<< ' ' << std::setw(5) << hw.high()
				<< ' ' << std::setw(5) << hw.wide()
				<< "  Cells,Bytes:"
				<< ' ' << std::setw(5) << size()
				<< ' ' << std::setw(5) << byteSize()
				;
			return oss.str();
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

		/*! \brief Content formatted with formating function
		 *
		 * Format is:
		 * \arg infoString() as first line
		 * \arg each line is a row of values formatted with fmtFunc
		 * \arg fieldSeparator is inserted between fields
		 * \arg rows are separated with a newline ('\n')
		 *
		 * Example:
		 * \snippet test_rasGrid.cpp DoxyExample00
		 */
		inline
		std::string
		infoStringContents
			( std::string const & title
				//!< Heading to print (along with size info)
			, std::function<std::string(Type const & elem)> const & fmtFunc
				//!< Function providing a string for each cell element
			, std::string const & fieldSeparator = std::string(" ")
				//!< Insert this string between individual fields
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
						oss << fieldSeparator << fmtFunc(*iter++);
					}
				}
			}
			else
			{
				oss << " <null>";
			}
			return oss.str();
		}

		//! General nearly(whatever) operation
		template <typename Func>
		inline
		bool
		nearlyFunc
			( ras::Grid<Type> const & other
			, Func const & func
			) const
		{
			bool same
				{  isValid() && other.isValid()
				&& (hwSize() == other.hwSize())
				};
			if (same)
			{
				same = std::equal(cbegin(), cend(), other.cbegin(), func);
			}
			return same;
		}
			

		//! True if *all* individual elements are nearly same within tol
		inline
		bool
		nearlyEquals
			( ras::Grid<Type> const & other
			, double const & tol = std::numeric_limits<Type>::epsilon()
			) const
		{
			return nearlyFunc
				( other
				, [&tol] (Type const & vA, Type const & vB)
					{ return engabra::g3::nearlyEquals(vA, vB, tol); }
				);
		}

		//! True if *all* individual elements are nearly same within tol
		inline
		bool
		nearlyEqualsAbs
			( ras::Grid<Type> const & other
			, double const & tol = std::numeric_limits<Type>::epsilon()
			) const
		{
			return nearlyFunc
				( other
				, [&tol] (Type const & vA, Type const & vB)
					{ return engabra::g3::nearlyEqualsAbs(vA, vB, tol); }
				);
		}

	};


} // [ras]

} // [quadloco]


namespace
{
	//! Put obj.infoString() to stream
	template <typename Type >
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::ras::Grid<Type> const & obj
		)
	{
		ostrm << obj.infoString();
		return ostrm;
	}

	//! True if the two grids are relatively equal within tolerance
	template <typename Type>
	inline
	bool
	nearlyEquals
		( quadloco::ras::Grid<Type> const & itemA
		, quadloco::ras::Grid<Type> const & itemB
		, double const & tol = std::numeric_limits<Type>::epsilon()
		)
	{
		return itemA.nearlyEquals(itemB, tol);
	}

	//! True if the two grids are absolutely equal within tolerance
	template <typename Type>
	inline
	bool
	nearlyEqualsAbs
		( quadloco::ras::Grid<Type> const & itemA
		, quadloco::ras::Grid<Type> const & itemB
		, double const & tol = std::numeric_limits<Type>::epsilon()
		)
	{
		return itemA.nearlyEqualsAbs(itemB, tol);
	}

} // [anon/global]

