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


#include "imgArea.hpp"
#include "imgSpot.hpp"
#include "valSpan.hpp"

#include <Engabra>

#include <array>
#include <cmath>
#include <numbers>
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
		img::Area const theArea{};

		//! If true clip the background patches into triangles
		bool const theWithTriangle{ false };

		//! If true, quadSignalAt() returns dithered background values
		bool const theWithSurround{ true };

		//! Area symmetric about origin with theEdgeMag size on each side
		inline
		static
		img::Area
		areaFor
			( double const & fullEdgeMag
			)
		{
			double const halfEdgeMag{ .5 * fullEdgeMag };
			return img::Area
				{ val::Span{ -halfEdgeMag,  halfEdgeMag }
				, val::Span{ -halfEdgeMag,  halfEdgeMag }
				};
		}

	public:

		//! Rendering options
		enum OptionFlags
		{
			  None            = 0x0000
			, WithSurround    = 0x0001
			, WithTriangle    = 0x0002
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
				( WithSurround
			//	| WithTriangle
				)
				//!< Or'd combo of quadloco::obj::QuadTarget::OptionFlags
			)
			: theEdgeMag{ fullEdgeLength }
			, theArea{ areaFor(theEdgeMag) }
			, theWithTriangle{ isSet(options, WithTriangle) }
			, theWithSurround{ isSet(options, WithSurround) }
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
		val::Span const &
		span0
			() const
		{
			return theArea.theSpans[0];
		}

		//! Span of coordinates in second dimension (e.g. bottom to top)
		inline
		val::Span const &
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
			constexpr double rootHalf{ .5 * std::numbers::sqrt2_v<double> };
			return (rootHalf * theEdgeMag);
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
		img::Spot
		centerSpot
			() const
		{
			return img::Spot{ 0., 0. };
		}

		//! Point on outer edge along 'x' axis
		inline
		img::Spot
		midSidePosX
			() const
		{
			return img::Spot{ halfEdgeMag(), 0. };
		}

		//! Point on outer edge along 'x' axis
		inline
		img::Spot
		midSideNegX
			() const
		{
			return img::Spot{ -halfEdgeMag(), 0. };
		}

		//! Point on outer edge along 'x' axis
		inline
		img::Spot
		midSidePosY
			() const
		{
			return img::Spot{ 0., halfEdgeMag() };
		}

		//! Point on outer edge along 'x' axis
		inline
		img::Spot
		midSideNegY
			() const
		{
			return img::Spot{ 0., -halfEdgeMag() };
		}

		//! The four outer corners in order: RT, LT, LB, RB
		inline
		std::array<img::Spot, 4u>
		cornerLocs
			() const
		{
			double const lft{ span0().theBeg };
			double const rgt{ span0().theEnd };
			double const bot{ span1().theBeg };
			double const top{ span1().theEnd };
			return std::array<img::Spot, 4u>
				{ img::Spot{ rgt, top }
				, img::Spot{ lft, top }
				, img::Spot{ lft, bot }
				, img::Spot{ rgt, bot }
				};
		}

		//! Create an arbitrary surround background scene
		inline
		double
		surroundSignalAt
			( img::Spot const & spotOnQuad
			) const
		{
			double const val0{ theMeanValue };

			// generate a signal relative to val0
			double delta{ 0. };
			double const amp{ .5 * std::abs(theWhite - theBlack) };
			double const frq{ .5 * engabra::g3::turnFull / radiusInner() };
			if (::isValid(spotOnQuad))
			{
				// ensure that delta is positive
				// (to avoid introducing negative pixel values)
				constexpr double rootHalf{ .5 * std::numbers::sqrt2_v<double> };
				delta = amp * rootHalf *
					( std::cos(frq*spotOnQuad[0])
					+ std::sin(frq*spotOnQuad[1])
					);
			}

			// noise to add
			static std::mt19937 gen(55243674u);
			static std::normal_distribution<double> distro(.0, .125);
			double const noise{ distro(gen) };

			// ensure positive value with fabs()
			double const signal{ std::fabs(val0 + delta + noise) };
			return signal;
		}


		//! Ideal radiometric intensity value at spot location
		inline
		double
		quadSignalAt
			( img::Spot const & spotOnQuad
			) const
		{
			// default to nan for outside of target area
			double value{ std::numeric_limits<double>::quiet_NaN() };
			if (theArea.contains(spotOnQuad))
			{
				// this code effectively defines the pattern
				// (ideally aligned with coordinate axes)
				double const & loc0 = spotOnQuad[0];
				double const & loc1 = spotOnQuad[1];

				// determine quadrant via signs (++, +-, --, -+)
				// a useful case for tertiary conditional operator :-)
				double const sign0{ (loc0 < 0.) ? -1. : 1. };
				double const sign1{ (loc1 < 0.) ? -1. : 1. };
				double const signEval{ sign0 * sign1 };
				value = (signEval < 0.) ? theWhite : theBlack;

				// introduce triangle clipping
				if (theWithTriangle && (theBlack == value))
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
			}
			else
			if (theWithSurround)
			{
				// requested to filling surrounding area
				value = surroundSignalAt(spotOnQuad);
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

