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
 * \brief Declarations for quadloco::ras::PeakRCV namespace
 *
 */


#include "QuadLoco/rasRowCol.hpp"

#include <Engabra>

#include <iostream>
#include <limits>
#include <sstream>
#include <string>


namespace quadloco
{

namespace ras
{

	//! \brief Peak Row,Col,Value
	struct PeakRCV
	{
		//! Location of peak (within assumed raster grid)
		ras::RowCol theRowCol;
		//! Value (e.g elevation) of peak
		double theValue{ engabra::g3::null<double>() };

		//! True if this instance has valid content
		inline
		bool
		isValid
			() const
		{
			return engabra::g3::isValid(theValue);
		}

		//! Update self to track maximum value
		inline
		void
		updateToMax
			( std::size_t const & row
			, std::size_t const & col
			, double const & value
			)
		{
			if ((! isValid()) || (theValue < value))
			{
				theRowCol = ras::RowCol{ row, col };
				theValue = value;
			}
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
			if (isValid())
			{
				oss
					<< "row,col,val:"
					<< ' ' << theRowCol
					<< ' ' << engabra::g3::io::fixed(theValue)
					;
			}
			else
			{
				oss << " <null>";
			}

			return oss.str();
		}

		//! True if this peak value is less than other.theValue
		inline
		bool
		operator<
			( PeakRCV const & other
			) const
		{
			return (theValue < other.theValue);
		}

	}; // PeakRCV


} // [ras]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::ras::PeakRCV const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::ras::PeakRCV const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

