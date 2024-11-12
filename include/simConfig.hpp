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


#include "datFence.hpp"
#include "datSizeHW.hpp"
#include "imgCamera.hpp"
#include "objQuadTarget.hpp"

#include <Rigibra>

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
		img::Camera const theCamera{};

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
		img::Camera const &
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

		//! Ciewing obj::QuadTarget face-on exactly filling camera format
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
			quadloco::dat::SizeHW const format{ numPixHW, numPixHW };
			double const pdToMatch{ (double)numPixHW };
			quadloco::img::Camera const camera{ format, pdToMatch };

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

