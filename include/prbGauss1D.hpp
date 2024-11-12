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
 * \brief Declarations for quadloco::prb::Gauss1D
 *
 */


#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <sstream>
#include <string>

#include <Engabra>


namespace quadloco
{

namespace prb
{

	//! Gaussian probability distribution in 1-dimension
	struct Gauss1D
	{
		double const theMean{ std::numeric_limits<double>::quiet_NaN() };
		double const theSigma{ std::numeric_limits<double>::quiet_NaN() };

		double const theCoeff{ std::numeric_limits<double>::quiet_NaN() };


		//! Normalizing coefficient for sigma value
		inline
		static
		double
		ampForSigma
			( double const & sigma
			)
		{
			static double const rootTwoPi
				{ std::sqrt(2. * std::numbers::pi_v<double>) };
			return (1. / (sigma * rootTwoPi));
		}

		// Construct invalid instance
		inline
		explicit
		Gauss1D
			() = default;

		//! Construct with expected mean value and standard deviation
		inline
		explicit
		Gauss1D
			( double const & mean
			, double const & sigma
			)
			: theMean{ mean }
			, theSigma{ sigma }
			, theCoeff{ ampForSigma(sigma) }
		{ }

		//! True if this instance has valid data
		inline
		bool
		isValid
			() const
		{
			return
				(  engabra::g3::isValid(theMean)
				&& engabra::g3::isValid(theSigma)
				);
		}

		//! Probability density value
		inline
		double
		operator()
			( double const & evalAt
			) const
		{
			double const delta{ (evalAt - theMean) };
			double const zVal{ delta / theSigma };
			double const arg{ -.5 * zVal * zVal };
			double const density{ theCoeff * std::exp(arg) };
			return density;
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
				<< "mean: " << engabra::g3::io::fixed(theMean)
				<< ' '
				<< "sigma: " << engabra::g3::io::fixed(theSigma)
				;

			return oss.str();
		}

	}; // Gauss1D


} // [prb]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::prb::Gauss1D const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::prb::Gauss1D const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

