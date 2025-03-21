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
 * \brief Declarations for quadloco::ops::Fence
 *
 */


#include <algorithm>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>


namespace quadloco
{

namespace ops
{

	//! Fence object that expands to contains added values.
	template <typename Type>
	struct Fence
	{
		//! Number of sample included
		std::size_t theCount{ 0u };
		//! Minimum of all samples considered (or NaN if none)
		Type theMin{ std::numeric_limits<Type>::max() };
		//! Maximum of all samples considered (or NaN if none)
		Type theMax{ std::numeric_limits<Type>::lowest() };

		//! True if this instance is contains valid data (and not empty)
		inline
		bool
		isValid
			()
		{
			return (! (theMax < theMin));
		}

		//! Exampand the fence to include also this value.
		inline
		void
		include
			( Type const & value
			)
		{
			if (0 == theCount)
			{
				theMin = value;
				theMax = value;
			}
			else
			{
				theMin = std::min(theMin, value);
				theMax = std::max(theMax, value);
			}
			++theCount;
		}

		//! Start of fence that contains all included values
		inline
		Type const &
		min
			() const
		{
			return theMin;
		}

		//! End of fence that contains all included values
		inline
		Type const &
		max
			() const
		{
			return theMax;
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
				<< "min: " << std::fixed << theMin
				<< ' '
				<< "max: " << std::fixed << theMax
				<< ' '
				<< "rng: " << std::fixed << (theMax - theMin)
				;
			return oss.str();
		}

	}; // Fence

} // [ops]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	template <typename Type>
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::ops::Fence<Type> const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	template <typename Type>
	inline
	bool
	isValid
		( quadloco::ops::Fence<Type> const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

