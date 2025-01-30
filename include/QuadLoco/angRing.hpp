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
 * \brief Declarations for quadloco::ang::Ring
 *
 */


#include "QuadLoco/ang.hpp"

#include <Engabra>

#include <cmath>
#include <limits>
#include <numbers>
#include <sstream>
#include <string>
#include <vector>


namespace quadloco
{

namespace ang
{

	//! \brief Circular buffer for managing angle data in binned arrays
	class Ring
	{
		std::size_t const theNumBins{ 0u };

		// cached data
		double const theAngPerBin{ engabra::g3::null<double>() };
		double const theBinPerAng{ engabra::g3::null<double>() };
	
	public:

		//! Create a null instance
		inline
		explicit
		Ring
			() = default;

		//! Construct full turn angle buffer with numBins cells
		inline
		explicit
		Ring
			( std::size_t const & numBins
			)
			: theNumBins{ numBins }
			, theAngPerBin{ ang::piTwo() / static_cast<double>(theNumBins) }
			, theBinPerAng{ 1. / theAngPerBin }
		{ }

		//! True if this instance has reasonable data (is not null)
		inline
		bool
		isValid
			() const
		{
			return
				(  (0u < theNumBins)
				&& (0. < theAngPerBin)
				&& (0. < theBinPerAng)
				);
		}

		//! Number of bins in underlying buffer
		inline
		std::size_t const &
		size
			() const
		{
			return theNumBins;
		}

		//! Angle span of a single bin (2*pi == size()*angleDelta())
		inline
		double const &
		angleDelta
			() const
		{
			return theAngPerBin;
		}

		//! Bin index associated with angle value
		inline
		std::size_t
		indexFor
			( double const & angle
			) const
		{
			double const mainAngle{ ang::principalAngle(angle) };
			double const deltaAngle{ mainAngle + ang::piOne() };
			double const dubBin{ theBinPerAng * deltaAngle };
			std::size_t const ndx
				{ static_cast<std::size_t>(std::floor(dubBin)) };
			return ndx;
		}

		//! Index that is (+/-)ndxDelta bins away in (circular)buffer.
		inline
		std::size_t
		indexRelativeTo
			( std::size_t const & refNdx
			, int const & deltaNdx
			) const
		{
			int advance{ deltaNdx };
			// if previous: advance increment by buffer size until positive
			while (advance < 0)
			{
				advance += size();
			}
			// for positive increment advance modulo buffer size
			std::size_t const ndx{ (refNdx + advance) % size() };
			return ndx;
		}

		//! Angle value associated with (start of) bin ndx
		inline
		double
		angleAtIndex
			( std::size_t const & ndx
			) const
		{
			double const deltaAngle{ theAngPerBin * static_cast<double>(ndx) };
			double const mainAngle{ deltaAngle - ang::piOne() };
			return mainAngle;
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
				<< "NumBins: " << theNumBins
				<< ' '
				<< "theAngPerBin: " << theAngPerBin
				<< ' '
				<< "theBinPerAng: " << theBinPerAng
				;
			return oss.str();
		}


	}; // Ring


} // [ang]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::ang::Ring const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::ang::Ring const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

