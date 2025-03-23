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
 * \brief Declarations for quadloco::img::QuadTarget
 *
 */


#include "QuadLoco/ang.hpp"
#include "QuadLoco/imgSpot.hpp"

#include <Engabra>

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>


namespace quadloco
{

namespace img
{
	//! Geometric description for perspective image of an obj::QuadTarget
	struct QuadTarget
	{
		// minimum image information for saying this is a quad target
		// (e.g. all outer edges could be cropped from image sample)

		//! \brief Center is required
		img::Spot theCenter{};

		/*! \brief One of two *UNITARY* directions - target "x-axis".
		 *
		 * This direction is associated with one of the two radial
		 * edges that has foreground (white) to the right side of
		 * the edge, and background (dark) to the left of the edge.
		 */
		img::Vector<double> theDirX{};

		//! \brief Second of two *UNITARY* directions - target "y-axis".
		img::Vector<double> theDirY{};

		//! Uncertainty associated with theCenter location
		double theCenterSigma{ std::numeric_limits<double>::quiet_NaN() };


		//! True if this instance contains valid data (not null)
		inline
		bool
		isValid
			() const
		{
			return
				(  theCenter.isValid()
				&& theDirX.isValid()
				&& theDirY.isValid()
				);
		}

		//! Location of intersection of axes
		inline
		img::Spot
		centerSpot
			() const
		{
			return theCenter;
		}

		//! Scalar uncertainty in centerSpot() location (from construction)
		inline
		double const &
		centerSigma
			() const
		{
			return theCenterSigma;
		}

		//! Direction of 'X-axis" (with a background spot to left)
		inline
		img::Vector<double> const &
		dirX
			() const
		{
			return theDirX;
		}

		//! Direction of 'Y-axis" (with a background spot to right)
		inline
		img::Vector<double> const &
		dirY
			() const
		{
			return theDirY;
		}

		//! True if theDirX(wedge)theDirY is aligned with e12
		inline
		bool
		isDextral
			() const
		{
			double const bivXY{ outer(theDirX, theDirY) };
			return (0. < bivXY);
		}

		//! True if this instance isValid() with (tol<angleSizeYwX)
		inline
		bool
		isStable
			( double const tol = std::numeric_limits<double>::epsilon()
			) const
		{
			bool stable{ isValid() };
			if (stable)
			{
				stable &= (tol < std::abs(angleSizeYwX()));
			}
			return stable;
		}

		//! Scalar (signed) angle from theDirX toward theDirY
		inline
		double
		angleSizeYwX
			() const
		{
			double const comp0{ dot(theDirX, theDirY) };
			double const comp1{ outer(theDirX, theDirY) };
			return ang::atan2(comp1, comp0);
		}

		//! True if this is same as other within numeric tolerance
		inline
		bool
		nearlyEquals
			( img::QuadTarget const & other
			, double const tolLoc = std::numeric_limits<double>::epsilon()
			, double const tolAng = std::numeric_limits<double>::epsilon()
			) const
		{
			bool same{ isValid() && other.isValid() };
			if (same)
			{
				// centers need to be the same
				bool const okayCenter
					{ ::nearlyEquals(theCenter, other.theCenter, tolLoc) };
				same &= okayCenter;
				if (same)
				{
					// check directions - primary direction alignment first
					bool okayDir{ false };
					double const tolSin{ std::abs(std::sin(tolAng)) };
					double const gotSinX{ outer(theDirX, other.theDirX) };
					if (std::abs(gotSinX) < tolSin)
					{
						// primary direction is either aligned or anti-aligned
						// so turn attention the secondary radial direction

						// decide wheter to test positive or negative secondary
						double const dotX{ dot(theDirX, other.theDirX) };

						// if primary direction is aligned, ...
						// ... then test positive secondary direction
						img::Vector<double> tstY{ other.theDirY };
						if (dotX < 0.)
						{
							// if primary direction is anti-aligned, ...
							// ... then test negative secondary direction
							tstY = -(other.theDirY);
						}

						// evaluate secondary direction alignemnt
						double const gotSinY{ outer(theDirY, tstY) };
						if (std::abs(gotSinY) < tolSin)
						{
							okayDir = same;
						}
					}
					same &= okayDir;
				}
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
				oss << title << '\n';
			}
			oss
				<< "isValid: " << std::boolalpha << isValid()
					<< "  isStable: " << std::boolalpha << isStable()
					<< "  isDextral: " << std::boolalpha << isDextral()
				<< '\n'
				<< "center(r,c): " << centerSpot()
					<< ' '
					<< "sigma: " << engabra::g3::io::fixed(centerSigma())
				<< '\n'
				<< "  dirX(r,c): " << dirX()
				<< '\n'
				<< "  dirY(r,c): " << dirY()
				;
			return oss.str();
		}

	}; // QuadTarget


} // [img]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::img::QuadTarget const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::img::QuadTarget const & item
		)
	{
		return item.isValid();
	}

	//! True if itemA and itemB are same within tolerance
	inline
	bool
	nearlyEquals
		( quadloco::img::QuadTarget const & itemA
		, quadloco::img::QuadTarget const & itemB
		, double const tolLoc = std::numeric_limits<double>::epsilon()
		, double const tolAng = std::numeric_limits<double>::epsilon()
		)
	{
		return itemA.nearlyEquals(itemB, tolLoc, tolAng);
	}


} // [anon/global]

