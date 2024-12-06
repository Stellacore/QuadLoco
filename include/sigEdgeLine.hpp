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
 * \brief Declarations for quadloco::sig::EdgeLine namespace
 *
 */


#include "ang.hpp"
#include "imgRay.hpp"
#include "imgSpot.hpp"
#include "sigItemWgt.hpp"

#include <Engabra>

#include <limits>
#include <ostream>
#include <sstream>
#include <string>


namespace quadloco
{

namespace sig
{
	//! Minimum edgeRay/line moment to consider line as significant
	constexpr double sMinMomentMag{ .75 };

	//! \brief Radially directed line (from origin to infinity).
	class EdgeLine
	{
	public:

		//! Start of this line
		img::Spot theStartSpot{};
		//! Edge ray: .start() defines line, .dir() defines turning moment
		img::Ray theEdgeRay{};

		// cached data

		//! Angle of line
		double theAngle{ std::numeric_limits<double>::quiet_NaN() };
		//! Turning moment of edgeRay direction wrt lineDir [-1,+1]
		double theMoment{ std::numeric_limits<double>::quiet_NaN() };

	public:

		//! Construct from a central spot and ray direction
		inline
		static
		EdgeLine
		from
			( img::Spot const & startSpot
			, img::Ray const & edgeRay
			)
		{
			img::Vector<double> const lineDir
				{ direction(edgeRay.start() - startSpot) };
			double const theta{ ang::atan2(lineDir[1], lineDir[0]) };
			double const dTheta{ outer(lineDir, edgeRay.direction()) };
			return EdgeLine{ startSpot, edgeRay, theta, dTheta };
		}

		//! True if members have valid data
		inline
		bool
		isValid
			() const
		{
			return
				(  engabra::g3::isValid(theAngle)
				&& engabra::g3::isValid(theMoment)
				);
		}

		//! Angle of this radial line
		inline
		double const &
		angleOfLine
			() const
		{
			return theAngle;
		}

		//! Spot location on unit circle at angleOfLine()
		inline
		img::Spot
		spotOnUnitCircle
			() const
		{
			return img::Spot{ lineDirection() };
		}

		//! Vector direction associated with angleOfLine()
		inline
		img::Vector<double>
		lineDirection
			() const
		{
			return img::Spot
				{ std::cos(theAngle)
				, std::sin(theAngle)
				};
		}

		//! The value of the turning moment (edge tangency)
		inline
		double const &
		turnMoment
			() const
		{
			return theMoment;
		}

		//! True if the magnitude of turnMoment() is larger than minMomentMag
		inline
		bool
		hasGoodMoment
			( double const & minMomentMag = sMinMomentMag
			) const
		{
			return (minMomentMag < std::abs(turnMoment()));
		}

		//! True if this and other have opposing direction of turning moments
		inline
		bool
		isTurnDirOppositeTo
			( EdgeLine const & other
			) const
		{
			return ((theMoment * other.theMoment) < 0.);
		}

		//! True if this and other have same direction of turning moments
		inline
		bool
		isTurnDirSameAs
			( EdgeLine const & other
			) const
		{
			return (! isTurnDirOppositeTo(other));
		}

		//! Index for EdgeLine most nearly opposing this current one
		inline
		NdxWgt
		opposingNdxWgt
			( std::vector<EdgeLine> const & others
			, std::size_t const & ndxCurr
			, double const & angSigma
			) const
		{
			NdxWgt nwMin{};
			if (hasGoodMoment())
			{
				// use points on unit circle to perform proximity math
				// to bipass having to deal with angle phase wrapping
				img::Spot const currDirSpot{ spotOnUnitCircle() };
				// find other unit locations near to this anti podal point
				img::Spot const antiDirSpot{ -currDirSpot };

				// compare self with each other instance and
				// find minimum line angle difference, but
				// only for other with same turning direction
				std::size_t ndxMin{ std::numeric_limits<std::size_t>::max() };
				double distMin{ std::numeric_limits<double>::max() };
				for (std::size_t ndx{0u} ; ndx < others.size() ; ++ndx)
				{
					if (ndxCurr != ndx)
					{
						EdgeLine const & other = others[ndx];
						if (  other.hasGoodMoment()
						   && this->isTurnDirSameAs(other)
						   )
						{
							img::Spot const unitDirSpot
								{ other.spotOnUnitCircle() };
							double const dist
								{ magnitude(antiDirSpot - unitDirSpot) };
							if (dist < distMin)
							{
								ndxMin = ndx;
								distMin = dist;
							}
						}
					}
				}

				if (ndxMin < others.size())
				{
					// for small distancs, dist is approximately the angle diff
					double const & angDiff = distMin;
					double const arg{ angDiff / angSigma };
					double const wgt{ std::exp(-arg*arg) };
					nwMin = NdxWgt{ ndxMin, wgt };
				}
			}
			return nwMin;
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
				<< "angleOfLine: " << engabra::g3::io::fixed(theAngle)
				<< ' '
				<< "edgeMoment: " << engabra::g3::io::fixed(theMoment)
				;

			return oss.str();
		}


	}; // EdgeLine


} // [sig]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::sig::EdgeLine const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::sig::EdgeLine const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

