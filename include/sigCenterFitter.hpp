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
 * \brief Declarations for quadloco::sig::CenterFitter namespace
 *
 */


#include "imgRay.hpp"
#include "imgSpot.hpp"
#include "imgVector.hpp"
#include "sigSpotSigma.hpp"

#include <cmath>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>


namespace quadloco
{

namespace sig
{

	//! \brief Least square solver for point central to multiple edgerays.
	class CenterFitter
	{
		// use first member to check validity
		double theAtA00{ std::numeric_limits<double>::quiet_NaN() };
		double theAtA01{ std::numeric_limits<double>::quiet_NaN() };
		//uble theAtA10{ }; // symmetric coefficient matrix
		double theAtA11{ std::numeric_limits<double>::quiet_NaN() };
		double theAtB0{ std::numeric_limits<double>::quiet_NaN() };
		double theAtB1{ std::numeric_limits<double>::quiet_NaN() };

		// track number of observations for dof adjustment in sigma estimate
		std::size_t theNumObs{ 0u };

	public:

		inline
		explicit
		CenterFitter
			() = default;

		//! True if this has valid data (although still may be singular)
		inline
		bool
		isValid
			() const
		{
			// just check first member
			return engabra::g3::isValid(theAtA00);
		}

		//! Incorporate ray into solution process
		inline
		void
		addRay
			( img::Ray const & ray
			, double const & wgt
			)
		{
			if (! isValid())
			{
				theAtA00 = 0.;
				theAtA01 = 0.;
				theAtA11 = 0.;
				theAtB0 = 0.;
				theAtB1 = 0.;
			}
			// access data: points on edge, edge dirs, and edge weights
			img::Vector<double> const & pnt = ray.start();
			img::Vector<double> const & dir = ray.direction();
			double const bj{ dot(dir, pnt) };
			// accumulate normal system
			theAtA00 += wgt * dir[0]*dir[0];
			theAtA01 += wgt * dir[0]*dir[1];
			//eAtA10 += wgt * dir[1]*dir[0]; // symmetric
			theAtA11 += wgt * dir[1]*dir[1];
			theAtB0  += wgt * dir[0] * bj;
			theAtB1  += wgt * dir[1] * bj;
			++theNumObs;
		}

		//! Least-Square spot location with max eigenvalue weight
		inline
		SpotSigma
		solutionSpotSigma
			() const
		{
			SpotSigma spotSigma{};

			// coefficient matrix
			double const & fwd00 = theAtA00;
			double const & fwd01 = theAtA01;
			double const & fwd10 = theAtA01; // theAtA10; // symmetric
			double const & fwd11 = theAtA11;

			// determinant
			double const det{ fwd00*fwd11 - fwd01*fwd10 };
			if (std::numeric_limits<double>::epsilon() < std::abs(det))
			{
				// inverse normal matrix
				double const scl{ 1. / det };
				double const inv00{  scl*fwd11 };
				double const inv01{ -scl*fwd01 };
				//uble const inv10{ -scl*fwd10 }; // symmetric
				double const inv11{  scl*fwd00 };

				// least square spot solution
				double const & inv10 = inv01; // symmetric
				img::Spot const solnSpot
					{ inv00*theAtB0 + inv01*theAtB1
					, inv10*theAtB0 + inv11*theAtB1
					};
				// estimated uncertainty in center point
				double const solnSigma
					{ SpotSigma::sigmaFromCovar
						(inv00, inv01, inv10, inv11, theNumObs)
					};
				// package for return
				spotSigma = SpotSigma{ solnSpot, solnSigma };
			}
			return spotSigma;
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
			using engabra::g3::io::fixed;
			double const & theAtA10 = theAtA01; // symmetric
			oss
				<< "  "
				<< "matAtA(0*):  "
					<< ' ' << fixed(theAtA00)
					<< ' ' << fixed(theAtA01)
					<< "  "
				<< "matAtB(0*):  "
					<< ' ' << fixed(theAtB0)
				<< '\n'
				<< "  "
				<< "matAtA(1*):  "
					<< ' ' << fixed(theAtA10)
					<< ' ' << fixed(theAtA11)
					<< "  "
				<< "matAtB(1*):  "
					<< ' ' << fixed(theAtB1)
				<< '\n';
				;
			oss << "soln: " << solutionSpotSigma().infoString();
			return oss.str();
		}

	}; // CenterFitter


} // [sig]

} // [quadloco]


namespace
{
	//! Put item.infoString() to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, quadloco::sig::CenterFitter const & item
		)
	{
		ostrm << item.infoString();
		return ostrm;
	}

	//! True if item is not null
	inline
	bool
	isValid
		( quadloco::sig::CenterFitter const & item
		)
	{
		return item.isValid();
	}

} // [anon/global]


