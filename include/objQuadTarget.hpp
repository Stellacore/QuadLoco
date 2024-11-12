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
#include <random>


namespace quadloco
{

namespace obj
{

	/*! \brief Quad target geometry and signal intensity model
	 */
	class QuadTarget
	{
		static constexpr double theBlack{ 0. };
		static constexpr double theWhite{ 1. };
		static constexpr double theMeanValue{ .5 * (theBlack + theWhite) };

		//! Magnitude of edges on square bounding full quad target signal.
		double const theEdgeMag{ engabra::g3::null<double>() };

		//! Bounding (half open) area - distinguish target from surround
		dat::Area const theArea{};

		//! If true clip the background patches into triangles
		bool const theDoubleTriangle{ false };

		//! If true, quadSignalAt() returns dithered background values
		bool const theAddSurround{ true };

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

		//! Rendering options
		enum OptionFlags
		{
			  None            = 0x0000
			, AddSurround     = 0x0001
			, DoubleTriangle  = 0x0002
		};

	private:

		//! True if testVal bit is set within haveBits
		inline
		static
		bool
		isSet
			( unsigned const & haveBits
			, OptionFlags const & testVal
			)
		{
			unsigned const setVal{ haveBits & (unsigned)testVal };
			bool const hasBit (0u != setVal);
			return hasBit;
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
				//!< Length of center lines (twice a radial edge length)
			, unsigned const & options =
				( AddSurround
			//	| DoubleTriangle
				)
				//!< Specify rendering options (or'd OptionFlags value)
			)
			: theEdgeMag{ fullEdgeLength }
			, theArea{ areaFor(theEdgeMag) }
			, theDoubleTriangle{ isSet(options, DoubleTriangle) }
			, theAddSurround{ isSet(options, AddSurround) }
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

		//! Edge length along outside of square
		inline
		double const &
		edgeMag
			() const
		{
			return theEdgeMag;
		}

		//! Half outer edge - edge size of {back,fore}ground squares
		inline
		double
		halfEdgeMag
			() const
		{
			return (.5 * theEdgeMag);
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
			return halfEdgeMag();
		}

		//! Center location in target frame (origin == zero)
		inline
		constexpr
		dat::Spot
		centerSpot
			() const
		{
			return dat::Spot{ 0., 0. };
		}

		//! Point on outer edge along 'x' axis
		inline
		dat::Spot
		midSidePosX
			() const
		{
			return dat::Spot{ halfEdgeMag(), 0. };
		}

		//! Point on outer edge along 'x' axis
		inline
		dat::Spot
		midSideNegX
			() const
		{
			return dat::Spot{ -halfEdgeMag(), 0. };
		}

		//! Point on outer edge along 'x' axis
		inline
		dat::Spot
		midSidePosY
			() const
		{
			return dat::Spot{ 0., halfEdgeMag() };
		}

		//! Point on outer edge along 'x' axis
		inline
		dat::Spot
		midSideNegY
			() const
		{
			return dat::Spot{ 0., -halfEdgeMag() };
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

		//! Create an arbitrary surround background scene
		inline
		double
		surroundSignalAt
			( dat::Spot const & spotOnQuad
			) const
		{
			double const val0{ theMeanValue };

			// generate a signal relative to val0
			double delta{ 0. };
			double const amp{ std::abs(theWhite - theBlack) };
			double const frq{ engabra::g3::turnFull / radiusInner() };
			if (::isValid(spotOnQuad))
			{
				double delta
					{ amp * std::cos(frq*spotOnQuad[0])
					+ amp * std::sin(frq*spotOnQuad[1])
					};
			}

			// noise to add
			static std::mt19937 gen(55243674u);
			static std::normal_distribution<double> distro(.0, .125);
			double const noise{ distro(gen) };

			return (val0 + delta + noise);
		}


		//! Ideal radiometric intensity value at spot location
		inline
		double
		quadSignalAt
			( dat::Spot const & spotOnQuad
			) const
		{
			// default to nan for outside of target area
			double value{ std::numeric_limits<double>::quiet_NaN() };

			// unless requested to filling surrounding area
			if (theAddSurround)
			{
				value = surroundSignalAt(spotOnQuad);
			}

			if (theArea.contains(spotOnQuad))
			{
				// this code effectively defines the pattern
				// (ideally aligned with coordinate axes)
				double const & loc0 = spotOnQuad[0];
				double const & loc1 = spotOnQuad[1];

				// a useful case for tertiary conditional operator
				double const sign0{ (loc0 < 0.) ? -1. : 1. };
				double const sign1{ (loc1 < 0.) ? -1. : 1. };
				double const signEval{ sign0 * sign1 };
				value = (signEval < 0.) ? theWhite : theBlack;
			}

			// introduce triangle clipping
			if (theDoubleTriangle && (theBlack == value))
			{
				// apply foreground color to outer triangle areas
				// of background signal to produce a double-triangle
				// target signal
				double const dot{ spotOnQuad[0] + spotOnQuad[1] };
				double const diagLim{ radiusInner() };
				if (diagLim < std::abs(dot))
				{
					value = theWhite;
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

