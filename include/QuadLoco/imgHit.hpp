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
 * \brief Declarations for quadloco::img::Hit namespace
 *
 */


#include "QuadLoco/imgSpot.hpp"

#include <Engabra>

#include <iostream>
#include <limits>
#include <sstream>
#include <string>


namespace quadloco
{

namespace img
{

	//! \brief A 2D location plus significance and uncertainty for a feature.
	class Hit
	{
		//! A representative location (e.g. mode, mean, etc) of a feature
		img::Spot theSpot{};

		//! The significance (e.g. pseudo-probability) of being a feature
		double theSignificance{ std::numeric_limits<double>::quiet_NaN() };

		//! Uncertainty (e.g. expected circular error) in theSpot coordinates
		double theUncertainty{ std::numeric_limits<double>::quiet_NaN() };

	public:

		//! \brief Default construction of null instance (isValid()==false)
		inline
		Hit
			() = default;

		//! \brief Value constructor
		inline
		explicit
		Hit
			( img::Spot const & imgSpot
			, double const & significance
			, double const & uncertainty
			)
			: theSpot{ imgSpot }
			, theSignificance{ significance }
			, theUncertainty{ uncertainty }
		{ }

		//! \brief Convenience construction from integer row/col indices
		inline
		explicit
		Hit
			( std::size_t const & row
			, std::size_t const & col
			, double const & significance
			, double const & uncertainty
			)
			: Hit
				( img::Spot
					{ static_cast<double>(row), static_cast<double>(col) }
				, significance
				, uncertainty
				)
		{ }


		//! \brief True if this instance contains valid data
		inline
		bool
		isValid
			() const
		{
			return
				(  theSpot.isValid()
				&& engabra::g3::isValid(theSignificance)
				&& engabra::g3::isValid(theUncertainty)
				);
		}

		//! \brief Image spot for hit location
		inline
		img::Spot const &
		location
			() const
		{
			return theSpot;
		}

		//! \brief Significance of peak (e.g. probability a feature was found)
		inline
		double const &
		value
			() const
		{
			return theSignificance;
		}

		//! \brief Geometric uncertainty (e.g. expected radial error) in theSpot
		inline
		double const &
		sigma
			() const
		{
			return theUncertainty;
		}

		//! \brief Order by significance value
		inline
		bool
		operator<
			( Hit const & other
			) const
		{
			return (value() < other.value());
		}

		//! \brief True if location is nearly equal
		inline
		bool
		nearlySameLocation
			( Hit const & other
			, double const & tol = std::numeric_limits<double>::epsilon()
			) const
		{
			// use img::Spot compare
			return theSpot.nearlyEquals(other.theSpot, tol);
		}

		//! \brief True if all data members are nearly equal
		inline
		bool
		nearlyEquals
			( Hit const & other
			, double const & tol = std::numeric_limits<double>::epsilon()
			) const
		{
			return
				(  nearlySameLocation(other, tol)
				&& engabra::g3::nearlyEquals
					(theSignificance, other.theSignificance, tol)
				&& engabra::g3::nearlyEquals
					(theUncertainty, other.theUncertainty, tol)
				);
		}

		//! \brief Descriptive information about this instance.
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
			using engabra::g3::io::fixed;
			oss
				<< "location: " << location()
				<< ' '
				<< "value: " << fixed(value())
				<< ' '
				<< "sigma: " << fixed(sigma())
				;

			return oss.str();
		}


	}; // Hit


} // [img]

} // [quadloco]



namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::img::Hit const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::img::Hit const & item
		)
	{
		return item.isValid();
	}

	//! True if items are nearly equal
	inline
	bool
	nearlyEquals
		( quadloco::img::Hit const & itemA
		, quadloco::img::Hit const & itemB
		)
	{
		return itemA.nearlyEquals(itemB);
	}

} // [anon/global]

