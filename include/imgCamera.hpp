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
 * \brief Image data/handling top level namespace
 *
 */


#include "datSpot.hpp"
#include "datSizeHW.hpp"

#include <Engabra>


namespace quadloco
{

/*! \brief Structs and utilities for supporting image data and processing.
 */
namespace img
{
	//! True if value in half open range: [minInclude <= value < maxExclude]
	inline
	bool
	inRange
		( double const & minInclude
		, double const & value
		, double const & maxExclude
		)
	{
		bool const inside
			{  (! (value < minInclude))  // min <= value
			&&    (value < maxExclude)   // value < max
			};
		return inside;
	}

	/*! \brief Simple (ideal) perspective camera imaging model.
	 *
	 * This struct uses "image positive" convention - for which "exterior"
	 * and "interior" coordinate frames are considered coincident.
	 *
	 * The "z" axis points away from the exterior object space (e.g.
	 * points expressed in the camera exterior frame should generally
	 * have negative 'z' coordinate values.
	 */
	struct Camera
	{
		//! Detector format size in [pixels]
		dat::SizeHW const theFormat{};

		//! Principal distance in [pix] - assume centered on format
		double const thePD{ engabra::g3::null<double>() };

		//! True if this instance contains valid data
		inline
		bool
		isValid
			() const
		{
			return
				(  theFormat.isValid()
				&& engabra::g3::isValid(thePD)
				&& (0. < thePD) // enforce 'image positive' convention
				);
		}

		//! Spot at center of the format
		inline
		dat::Spot
		formatCenter
			() const
		{
			double const highDub{ (double)theFormat.high() };
			double const wideDub{ (double)theFormat.wide() };
			return dat::Spot{ (.5 * highDub), (.5 * wideDub) };
		}

		//! Projected spot in camera detector frame (but maybe out of bounds)
		inline
		dat::Spot
		projectedSpotFor
			( engabra::g3::Vector const & locInExt
			) const
		{
			dat::Spot spot{};
			if (engabra::g3::isValid(locInExt) && isValid())
			{
				double const & zInExt = locInExt[2];
				if (zInExt < 0.) // potentially in field of view
				{
					double const imgDist{ std::abs(thePD) };
					double const objDist{ std::abs(zInExt) };
					double const scalePixPerObj{ imgDist / objDist };
					double const dX{ scalePixPerObj * locInExt[0] };
					double const dY{ scalePixPerObj * locInExt[1] };
					dat::Spot const dRowCol
						{ -dY  // Row value gets less for larger object 'y'
						,  dX  // Col value tracks object 'x' values
						};
					// candidate location
					spot = dat::Spot{ formatCenter() + dRowCol };
				}
			}
			return spot;
		}

		//! Detector location expressed on camera detector
		inline
		dat::Spot
		detectorSpotFor
			( engabra::g3::Vector const & locInExt
			) const
		{
			dat::Spot spot{};
			dat::Spot const aSpot{ projectedSpotFor(locInExt) };
			if (aSpot.isValid())
			{
				// check if candidate is inside detector format
				double const highDub{ (double)theFormat.high() };
				double const wideDub{ (double)theFormat.wide() };
				bool const rowOnDet{ inRange(0., aSpot.row(), highDub) };
				bool const colOnDet{ inRange(0., aSpot.col(), wideDub) };
				if (rowOnDet && colOnDet)
				{
					spot = aSpot;
				}
			}
			return spot;
		}

		//! Descriptive information about this instance.
		inline
		std::string
		infoString
			( std::string const & title=std::string()
			) const
		{
			std::ostringstream oss;
			if (!title.empty())
			{
				oss << title << " ";
			}
			if (! isValid())
			{
				oss << " <null> ... ";
			}
			oss
				<< "fmt: " << theFormat
				<< ' '
				<< "pd: " << engabra::g3::io::fixed(thePD, 4u, 3u)
				;
			return oss.str();
		}

	}; // Camera


} // [img]

} // [quadloco]


namespace
{
	//! Put obj.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::img::Camera const & obj
		)
	{
		ostrm << obj.infoString();
		return ostrm;
	}

} // [anon/global]

