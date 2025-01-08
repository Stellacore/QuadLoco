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
 * \brief Declarations for quadloco::sim::Config
 *
 */


#include "objCamera.hpp"
#include "objQuadTarget.hpp"
#include "opsFence.hpp"
#include "rasChipSpec.hpp"
#include "rasSizeHW.hpp"

#include <Rigibra>

#include <array>
#include <cmath>
#include <sstream>
#include <string>


namespace quadloco
{

namespace sim
{

	/*! \brief Configuration info needed to simulate a QuadTarget image
	 *
	 * \snippet test_simSampler.cpp DoxyExample01
	 */
	struct Config
	{
		//! Target in the scene - defines the reference frame coordinates
		obj::QuadTarget const theObjQuad{};

		//! Camera for producing perspective image of target
		obj::Camera const theCamera{};

		//! Camera station (exterior frame) w.r.t. #theObjQuad own frame
		rigibra::Transform const theStaWrtQuad{};


		//! Object space target (comprising the scene to be simulated)
		inline
		obj::QuadTarget const &
		objQuadTarget
			() const
		{
			return theObjQuad;
		}

		//! Camera associated with this configuration
		inline
		obj::Camera const &
		camera
			() const
		{
			return theCamera;
		}

		//! Camera station (exterior frame) w.r.t. obj::QuadTarget frame
		inline
		rigibra::Transform const &
		xformStaWrtQuad
			() const
		{
			return theStaWrtQuad;
		}

		//! Detector spot on image (in format, else null) for spot on target
		inline
		img::Spot
		imgSpotForTgtSpot
			( img::Spot const & spotOnTgt
			) const
		{
			engabra::g3::Vector const pntInTgt{ spotOnTgt[0], spotOnTgt[1] };
			engabra::g3::Vector const pntInCam{ theStaWrtQuad(pntInTgt) };
			return theCamera.detectorSpotFor(pntInCam);
		}

		//! \brief Sub region of grid that containing entire quad image
		inline
		ras::ChipSpec
		chipSpecForQuad
			( std::size_t const & pad
				//!< Expand chip from quad corners by this much extra
			) const
		{
			// corners in obj::QuadTarget frame
			std::array<img::Spot, 4u> const objSpots{ theObjQuad.cornerLocs() };

			// corners in image (camera format) space
			std::array<img::Spot, 4u> const imgSpots
				{ imgSpotForTgtSpot(objSpots[0])
				, imgSpotForTgtSpot(objSpots[1])
				, imgSpotForTgtSpot(objSpots[2])
				, imgSpotForTgtSpot(objSpots[3])
				};

			// get boxed limits of image projections
			double dRowMin{ imgSpots[0].row() };
			double dColMin{ imgSpots[0].col() };
			double dRowMax{ imgSpots[0].row() };
			double dColMax{ imgSpots[0].col() };
			for (std::size_t nn{1u} ; nn < imgSpots.size() ; ++nn)
			{
				dRowMin = std::min(dRowMin, imgSpots[nn].row());
				dColMin = std::min(dColMin, imgSpots[nn].col());
				dRowMax = std::max(dRowMax, imgSpots[nn].row());
				dColMax = std::max(dColMax, imgSpots[nn].col());
			}

			// adjust by requested pad amount
			dRowMin -= pad;
			dColMin -= pad;
			dRowMax += pad;
			dColMax += pad;

			// adjust to ensure inside of overall grid
			dRowMin = std::max(dRowMin, 0.);
			dColMin = std::max(dColMin, 0.);
			dRowMax = std::min(dRowMax, (double)theCamera.theFormat.high());
			dColMax = std::min(dColMax, (double)theCamera.theFormat.wide());

			// update return
			ras::RowCol const rc0
				{ static_cast<std::size_t>(dRowMin)
				, static_cast<std::size_t>(dColMin)
				};
			ras::SizeHW const hwSize
				{ static_cast<std::size_t>(std::floor(dRowMax - dRowMin))
				, static_cast<std::size_t>(std::floor(dColMax - dColMin))
				};
			return ras::ChipSpec{ rc0, hwSize };
		}

		//! Viewing obj::QuadTarget face-on exactly filling camera format
		inline
		static
		Config
		faceOn
			( obj::QuadTarget const & objQuad
			, std::size_t const & numPixHW
			)
		{
			// configure camera and orientation such that
			// camera format exactly matches the objQuad target

			//
			// create a 1:1 camera geometry
			// * pd == high() == wide()
			//
			quadloco::ras::SizeHW const format{ numPixHW, numPixHW };
			double const pdToMatch{ (double)numPixHW };
			quadloco::obj::Camera const camera{ format, pdToMatch };

			//
			// exterior orientation - fit to obj quad target size
			// * directly above the quad target
			// * with object distance matching objQuad edge mag
			//   (so that 1:objQuadTarget unit == 1:imgDetector unit)
			//
			double const camOriZ{ objQuad.edgeMag() };
			engabra::g3::Vector const stationAbove{ 0., 0., camOriZ };
			rigibra::Transform const xCamWrtQuad
				{ stationAbove
				, rigibra::identity<rigibra::Attitude>()
				};

			return Config{ objQuad, camera, xCamWrtQuad };
		}


		//! True if this instance contains valid data
		inline
		bool
		isValid
			() const
		{
			return
				(  theCamera.isValid()
				&& rigibra::isValid(theStaWrtQuad)
				);
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
				<< "theStaWrtQuad: " << theStaWrtQuad
				<< '\n'
				<< "theCamera: " << theCamera
				;

			return oss.str();
		}


	}; // Config


} // [sim]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::sim::Config const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::sim::Config const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

