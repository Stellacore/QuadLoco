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

	/*! Configuration information needed to simulation an image of QuadTarget
	 *
	 * \snippet test_simSampler.cpp DoxyExample01
	 */
	struct Config
	{
		img::Camera const theCamera{};

		rigibra::Transform const theStaWrtQuad{};


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

