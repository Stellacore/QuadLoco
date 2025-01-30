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
 * \brief Declarations for quadloco::sys::Timer namespace
 *
 */


#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>


namespace quadloco
{

/*! \brief System/environment supplemental code in namespace quadloco::sys
 */
namespace sys
{

	//! \brief High precision (nanosecond) timer
	struct Timer
	{
		//! Instantate with a string value here if desired
		std::string theName{};

		//! Start of interval for elapsed() computation.
		std::chrono::time_point<std::chrono::steady_clock>
			theBeg{ std::chrono::steady_clock::now() };

		//! End of interval for elapsed() computation.
		std::chrono::time_point<std::chrono::steady_clock>
			theEnd{ std::chrono::steady_clock::now() };

		//! Set theBeg time value (used for elapsed time evaluation)
		inline
		void
		restart
			( std::string const & name = {}
			)
		{
			theBeg = std::chrono::steady_clock::now();
			theName = name;
		}

		//! Set theEnd time value (used for elapsed time evaluation)
		inline
		void
		stop
			()
		{
			theEnd = std::chrono::steady_clock::now();
		}

		//! Elapsed time from restart() to stop() (theEnd - theBeg) in [sec]
		inline
		double
		elapsed
			() const
		{
			using namespace std::chrono;
			// clang complains about before c++17 compatibility
		//	duration const duro{ duration_cast<nanoseconds>(theEnd - theBeg) };
			// clang complains about missing '::num' member
		//	duration<double, nanoseconds> const duro{ theEnd - theBeg };
			// clang creates who-knows-what kind of type
			auto const duro{ theEnd - theBeg };

			double const delta{ 1.e-9 * static_cast<double>(duro.count()) };

			return delta;
		}

		//! Descriptive information about this instance.
		inline
		std::string
		infoString
			( std::string const & title = {}
			, std::size_t const & numDigitAfter = 9u
			) const
		{
			std::ostringstream oss;
			if (! title.empty())
			{
				oss << title << ' ';
			}
			std::size_t fWide{ 4u + numDigitAfter };
			oss
				<< std::setw(fWide) << std::fixed
					<< std::setprecision(numDigitAfter)
					<< elapsed()
				<< ' '
				<< theName
				;

			return oss.str();
		}


	}; // Timer


} // [sys]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::sys::Timer const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	/*
	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::obj::QuadTarget const & item
		)
	{
		return item.isValid();
	}
	*/

} // [anon/global]

