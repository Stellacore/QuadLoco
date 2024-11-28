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
 * \brief Declarations for quadloco::img::Vector class template
 *
 */


#include <Engabra>

#include <array>
#include <cmath>
#include <format>


namespace quadloco
{

namespace img
{
	template <typename Type>
	inline
	bool
	isValidType
		( Type const & value
		)
	{
		constexpr Type zero{ 0 };
		return (std::isnormal(value) || (zero == value) );
	}

	//! Generic 2D vector structure
	template <typename Type>
	struct Vector
	{
		//! Component data values
		std::array<Type, 2u> theData
			{ engabra::g3::null<Type>()
			, engabra::g3::null<Type>()
			};

		//! Functor for formatting member data into a string
		struct Formatter
		{
			//! std::format to apply to each of the two theComps
			std::string const theFormatEach{ "{:4.1f}" };

			// String after theFormatEach to encode each of the two values
			inline
			std::string
			operator()
				( Vector<Type> const & elem
				) const
			{
				std::ostringstream fmt;
				fmt << '(' << theFormatEach << ',' << theFormatEach << ')';
				return std::vformat
					( fmt.str()
					, std::make_format_args(elem[0], elem[1])
					);
			}

		}; // Formatter


		inline
		~Vector
			() = default;

		//! True if this instance contain valid data (is not null)
		inline
		bool
		isValid
			() const
		{
			return
				(  isValidType<Type>(theData[0])
				&& isValidType<Type>(theData[1])
				);
		}

		//! Access to component values
		inline
		Type const &
		operator[]
			( std::size_t const & ndx
			) const
		{
			return theData[ndx];
		}

		//! True if this is same as other within tolerance
		inline
		bool
		nearlyEquals
			( Vector const & other
			, Type const tol = std::numeric_limits<Type>::epsilon()
			) const
		{
			return
				{  engabra::g3::nearlyEquals(theData[0], other.theData[0], tol)
				&& engabra::g3::nearlyEquals(theData[1], other.theData[1], tol)
				};
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
				<< engabra::g3::io::fixed(theData[0])
				<< ' '
				<< engabra::g3::io::fixed(theData[1])
				;

			return oss.str();
		}

	}; // Vector

	//! Scaling operations
	template <typename Type>
	inline
	Vector<Type>
	operator*
		( Type const & scl
		, Vector<Type> const & vec
		)
	{
		return { scl*vec[0], scl*vec[1] };
	}

	//
	// Vector properties
	//

	//! Scalar magnitude (length) of vec
	template <typename Type>
	inline
	Type
	magnitude
		( Vector<Type> const & vec
		)
	{
		return std::hypot(vec[0], vec[1]);
	}

	//! Unitary (1.==magnitude) direction for vec
	template <typename Type>
	inline
	Vector<Type>
	direction
		( Vector<Type> const & vec
		)
	{
		Vector<Type> unit{};
		Type const mag{ magnitude(vec) };
		if (engabra::g3::isValid(mag))
		{
			unit = (1./mag) * vec;
		}
		return unit;
	}


	//
	// Uniary
	//

	//! Difference (vecA - vecB)
	template <typename Type>
	inline
	Vector<Type>
	operator-
		( Vector<Type> const & vec
		)
	{
		return Vector<Type>{ -vec[0], -vec[1] };
	}


	//
	// Binary
	//

	//! Sum (vecA + vecB)
	template <typename Type>
	inline
	Vector<Type>
	operator+
		( Vector<Type> const & vecA
		, Vector<Type> const & vecB
		)
	{
		return Vector<Type>{ vecA[0] + vecB[0], vecA[1] + vecB[1] };
	}

	//! Difference (vecA - vecB)
	template <typename Type>
	inline
	Vector<Type>
	operator-
		( Vector<Type> const & vecA
		, Vector<Type> const & vecB
		)
	{
		return Vector<Type>{ vecA[0] - vecB[0], vecA[1] - vecB[1] };
	}

	//! Scalar dot product
	template <typename Type>
	inline
	Type
	dot
		( Vector<Type> const & vecA
		, Vector<Type> const & vecB
		)
	{
		return (vecA[0]*vecB[0] + vecA[1]*vecB[1]);
	}

	//! Real analog of wedge product
	template <typename Type>
	inline
	Type
	outer
		( Vector<Type> const & vecA
		, Vector<Type> const & vecB
		)
	{
		return (vecA[0]*vecB[1] - vecA[1]*vecB[0]);
	}

} // [img]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	template <typename Type>
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::img::Vector<Type> const & item
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
		( quadloco::img::Vector<Type> const & item
		)
	{
		return item.isValid();
	}

	//! True if both instances are same within tolerance
	template <typename Type>
	inline
	bool
	nearlyEquals
		( quadloco::img::Vector<Type> const & itemA
		, quadloco::img::Vector<Type> const & itemB
		, Type const tol = std::numeric_limits<Type>::epsilon()
		)
	{
		return itemA.nearlyEquals(itemB, tol);
	}

} // [anon/global]

