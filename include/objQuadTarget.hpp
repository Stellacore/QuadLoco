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
 * \brief Structures and functions representing object space quad targets
 *
 */


#include "datArea.hpp"
#include "datSpot.hpp"

#include <Engabra>

#include <array>
#include <cmath>


namespace quadloco
{

namespace obj
{

	/*! \brief
	 *
	 */
	class QuadTarget
	{
		//! Magnitude of edges on square bounding full quad target signal.
		double const theEdgeMag{ engabra::g3::null<double>() };

		//! Bounding (half open) area - distinguish target from surround
		dat::Area const theArea{};

		//! Area symmetric about origin with theEdgeMag size on each side
		inline
		static
		dat::Area
		areaFor
			( double const & fullEdgeMag
			)
		{
			double const halfEdgeMag{ .5 * fullEdgeMag };
			return dat::Area
				{ dat::Span{ -halfEdgeMag,  halfEdgeMag }
				, dat::Span{ -halfEdgeMag,  halfEdgeMag }
				};
		}

	public:

		//! Construct a null instance
		inline
		QuadTarget
			() = default;

		//! Construct a square target with specified edge sizes
		inline
		explicit
		QuadTarget
			( double const & fullEdgeLength
			)
			: theEdgeMag{ fullEdgeLength }
			, theArea{ areaFor(theEdgeMag) }
		{ }

		//! True if this instance contains valid data (is not null)
		inline
		bool
		isValid
			() const
		{
			return engabra::g3::isValid(theEdgeMag);
		}

		//! Span of coordinates in first dimension (e.g. left to right)
		inline
		dat::Span const &
		span0
			() const
		{
			return theArea.theSpans[0];
		}

		//! Span of coordinates in second dimension (e.g. bottom to top)
		inline
		dat::Span const &
		span1
			() const
		{
			return theArea.theSpans[1];
		}

		//! Center location in target frame (origin == zero)
		inline
		constexpr
		dat::Spot
		centerLoc
			() const
		{
			return dat::Spot{ 0., 0. };
		}

		//! Circumscribing radius (e.g. hits outer background corners)
		inline
		double
		radiusOuter
			() const
		{
			return (std::sqrt(.5) * theEdgeMag);
		}

		//! Inscribing radius (e.g. hits midside background corners)
		inline
		double
		radiusInner
			() const
		{
			return (.5 * theEdgeMag);
		}

		//! The four outer corners in order: RT, LT, LB, RB
		inline
		std::array<dat::Spot, 4u>
		cornerLocs
			() const
		{
			double const lft{ span0().theBeg };
			double const rgt{ span0().theEnd };
			double const bot{ span1().theBeg };
			double const top{ span1().theEnd };
			return std::array<dat::Spot, 4u>
				{ dat::Spot{ rgt, top }
				, dat::Spot{ lft, top }
				, dat::Spot{ lft, bot }
				, dat::Spot{ rgt, bot }
				};
		}

		//! Ideal radiometric intensity value at spot location
		inline
		float
		intensityAt
			( dat::Spot const & spotOnQuad
			) const
		{
			// default to nan for outside of target area
			float value{ std::numeric_limits<float>::quiet_NaN() };
			constexpr float black{ 0.f };
			constexpr float white{ 1.f };
			if (theArea.contains(spotOnQuad))
			{
				// this code effectively defines the pattern
				// (ideally aligned with coordinate axes)
				double const & loc0 = spotOnQuad[0];
				double const & loc1 = spotOnQuad[1];
				if (loc0 < 0.) // note half open interval convention
				{
					// on "left" half
					if (loc1 < 0.)
					{
						// on "bottom" half
						value = black;
					}
					else
					{
						// on "top" half
						value = white;
					}
				}
				else
				{
					// on "right" half
					if (loc1 < 0.)
					{
						// on "bottom" half
						value = white;
					}
					else
					{
						// on "top" half
						value = black;
					}
				}
			}
			return value;
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
				<< "edgeMag: " << engabra::g3::io::fixed(theEdgeMag)
				<< " area: " << theArea
				;

			return oss.str();
		}

	}; // QuadTarget


} // [obj]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::obj::QuadTarget const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::obj::QuadTarget const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

