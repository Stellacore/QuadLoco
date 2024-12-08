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
 * \brief Declarations for quadloco::sig::SpotSigma namespace
 *
 */


#include "imgSpot.hpp"

#include <Engabra>

#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>


namespace quadloco
{

namespace sig
{

	//! An img::Spot location plus uncertainty estimate
	struct SpotSigma
	{
		img::Spot theSpot{};
		double theSigma{ std::numeric_limits<double>::quiet_NaN() };

		//! Estimate center point RMSE values (from largest eigen vector)
		inline
		static
		double
		sigmaFromCovar
			( double const & covar00
			, double const & covar01
			, double const & covar10
			, double const & covar11
			, std::size_t const & numObs
			)
		{
			double sigma{ std::numeric_limits<double>::quiet_NaN() };
			// determinant (expect strictly positive for meaningful covar)
			double const det{ covar00*covar11 - covar10*covar01 };
			if (0. < det)
			{
				// covariance matrix eigen values are parameter variances
				// characteristic polynomial (quadratic coefficient == 1)
				double const beta{ -.5 * (covar00 + covar11) };
				double const & gamma = det;
				double const lamMid{ -beta };
				double const radicand{ beta*beta - gamma };
				if (! (radicand < 0.)) // true in theory (for proper code)
				{
					// compute eigen values
					double const root{ std::sqrt(radicand) };
					double const lamNeg{ lamMid - root };
					double const lamPos{ lamMid + root };
					double const lamBig
						{ std::max(std::abs(lamNeg), std::abs(lamPos)) };
					// standard deviation is root of eigen value
					if (1u < numObs)
					{
					// For estimating residual magnitudes
						// adjust eigen value by statistical dof multiplier
						// given numObs observations and 2 parameter freedoms
					//	double const dNumObs{ (double)numObs };
					//	double const meanMult{ dNumObs / (dNumObs - 2.) };
					//	double const var{ meanMult * lamBig };
					//	sigma = std::sqrt(var);
						// 
						// For estimating parameter uncertainty
						sigma = std::sqrt(lamBig);

						/*
						using engabra::g3::io::fixed;
						std::cout << "covar0*: "
							<< ' ' << fixed(covar00)
							<< ' ' << fixed(covar01)
							<< '\n';
						std::cout << "covar1*: "
							<< ' ' << fixed(covar10)
							<< ' ' << fixed(covar11)
							<< '\n';
						std::cout << "lamNeg: " << fixed(lamNeg) << '\n';
						std::cout << "lamPos: " << fixed(lamPos) << '\n';
						std::cout << " sigma: " << fixed(sigma) << '\n';
						*/

					}
				}
			}
			return sigma;
		}

		//! True if this instance contains valid data
		inline
		bool
		isValid
			() const
		{
			return theSpot.isValid();
		}

		//! Access to theSpot
		inline
		img::Spot const &
		spot
			() const
		{
			return theSpot;
		}

		//! Access to theSigma
		inline
		double const &
		sigma
			() const
		{
			return theSigma;
		}

		//! Weight associated with sigma value relative to expected uncertainty
		inline
		double
		weight
			( double const & expSigma = 1.
			) const
		{
			double const argZ{ sigma() / expSigma };
			return std::exp(-argZ*argZ);
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
				<< "spot: " << theSpot
				<< " sigma: " << engabra::g3::io::fixed(theSigma);
				;

			return oss.str();
		}


	}; // SpotSigma



} // [sig]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::sig::SpotSigma const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::sig::SpotSigma const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]

