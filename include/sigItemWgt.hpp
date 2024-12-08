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
 * \brief Declarations for quadloco::sig::ItemWgt namespace
 *
 */


#include "imgRay.hpp"
#include "imgSpot.hpp"
#include "sigQuadTarget.hpp"

#include <Engabra>

#include <limits>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>


namespace quadloco
{

namespace sig
{

	//! \brief An arbitrary type item instance and associated weight.
	template <typename ItemType>
	struct ItemWgt
	{
		ItemType theItem{};
		double theWeight{ std::numeric_limits<double>::quiet_NaN() };

		//! True if this instance contains valid data
		inline
		bool
		isValid
			() const
		{
			return engabra::g3::isValid(theWeight);
		}

		//! Item instance
		inline
		ItemType const &
		item
			() const
		{
			return theItem;
		}

		//! Weight value
		inline
		double const &
		weight
			() const
		{
			return theWeight;
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
				<< "item: " << theItem
				<< ' '
				<< "wgt: " << engabra::g3::io::fixed(theWeight);
				;
			return oss.str();
		}

	}; // ItemWgt

	//! Display a collection of ItemWgt types
	template <typename Type>
	inline
	std::string
	infoStringFor
		( std::vector<Type> const & itemWgts
		, std::string const & name
		)
	{
		std::ostringstream oss;
		oss << name << ".size: " << itemWgts.size();
		for (Type const & itemWgt : itemWgts)
		{
			oss << '\n' << name << ": " << itemWgt.infoString();
		}
		return oss.str();
	}


	//! An img::Ray item and associated weight
	using RayWgt = ItemWgt<img::Ray>;

	//! An img::Spot item and associated weight
	using SpotWgt = ItemWgt<img::Spot>;

	//! An index item and associated weight
	using NdxWgt = ItemWgt<std::size_t>;

	//! A sig::QuadTarget item and associated weight
	using QuadWgt = ItemWgt<sig::QuadTarget>;


	//! An angle item and associated weight
	struct AngleWgt : public ItemWgt<double>
	{
		// overload ItemWgt formatting (brute force, non-virtual)
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
				<< "item: " << engabra::g3::io::fixed(item())
				<< ' '
				<< "wgt: " << engabra::g3::io::fixed(weight())
				;
			return oss.str();
		}
	};



} // [sig]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	template <typename ItemType>
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::sig::ItemWgt<ItemType> const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	template <typename ItemType>
	inline
	bool
	isValid
		( quadloco::sig::ItemWgt<ItemType> const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

