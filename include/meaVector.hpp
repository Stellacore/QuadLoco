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
 * \brief Declarations for quadloco::mea::Vector namespace
 *
 */


#include "imgVector.hpp"
#include "meaCovar.hpp"
#include "opsmatrix.hpp"
#include "rasGrid.hpp"

#include <iostream>
#include <limits>
#include <sstream>
#include <string>


namespace quadloco
{

namespace mea
{

	//! \brief Location and uncertainty as normal probability distribution.
	class Vector
	{
		//! Best estimate of location
		img::Vector<double> theLoc{};

		//! Best estimate of uncertainty as standard covariance
		Covar theCovar{};

	public:

		//! Default construction of null instance (isValid() == false)
		inline
		explicit
		Vector
			() = default;

		//! Construct with location and identify covariance matrix
		inline
		explicit
		Vector
			( img::Vector<double> const & loc
			, double const & sigma
			)
			: theLoc{ loc }
			, theCovar(sigma)
		{ }

		//! Construct with location and identify covariance matrix
		inline
		explicit
		Vector
			( img::Vector<double> const & loc
			, ops::Matrix const & covar
			)
			: theLoc{ loc }
			, theCovar(covar)
		{ }

		//! True if this instance contains viable data
		inline
		bool
		isValid
			() const
		{
			return
				(  theLoc.isValid()
				&& theCovar.isValid()
				);
		}

		//! Best estimated location
		inline
		img::Vector<double> const &
		location
			() const
		{
			return theLoc;
		}

		//! Direct access to covariance structure
		inline
		Covar const &
		covar
			() const
		{
			return theCovar;
		}

		//! Estimated circular uncertainty (root-mean-square of semiaxes)
		inline
		double
		deviationRMS
			() const
		{
			return theCovar.deviationRMS();
		}

		//! True if this and other have same data within tolerance
		inline
		bool
		nearlyEquals
			( Vector const & other
			, double const & tol = std::numeric_limits<double>::epsilon()
			) const
		{
			bool same{ isValid() && other.isValid() };
			if (same)
			{
				ops::Matrix const matA{ theCovar.matrix() };
				ops::Matrix const matB{ other.theCovar.matrix() };
				using engabra::g3::nearlyEquals;
				same =
					(  theLoc.nearlyEquals(other.theLoc, tol)
					&& nearlyEquals(matA(0u, 0u), matB(0u, 0u))
					&& nearlyEquals(matA(0u, 1u), matB(0u, 1u))
					&& nearlyEquals(matA(1u, 0u), matB(1u, 0u))
					&& nearlyEquals(matA(1u, 1u), matB(1u, 1u))
					);
			}
			return same;
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
				<< theLoc
				<< '\n'
				<< theCovar
				;
			return oss.str();
		}

	}; // Vector


} // [mea]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::mea::Vector const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::mea::Vector const & item
		)
	{
		return item.isValid();
	}

	//! True if both instances have same data within tolerance.
	inline
	bool
	nearlyEquals
		( quadloco::mea::Vector const & itemA
		, quadloco::mea::Vector const & itemB
		, double const & tol = std::numeric_limits<double>::epsilon()
		)
	{
		return itemA.nearlyEquals(itemB, tol);
	}

} // [anon/global]

