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
 * \brief Declarations for quadloco::hough::AdderAD
 *
 */


#include "datGrid.hpp"
#include "datRowCol.hpp"
#include "datSizeHW.hpp"
#include "datSizeMap.hpp"
#include "datSpan.hpp"
#include "datSpot.hpp"
#include "houghParmAD.hpp"
#include "pix.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>


namespace quadloco
{

namespace hough
{

	//! Accumulation buffer for hough alpha,delta parameters
	class AdderAD
	{
		static constexpr double pi{ std::numbers::pi_v<double> };
		static constexpr double piTwo{ 2. * std::numbers::pi_v<double> };

		dat::SizeMap const theSizeMap;
		dat::Grid<float> theGridAD{};

		//! Add/subtract 2*pi if iAngle is under/over (min, max) range
		inline
		static
		double
		angleInRange
			( double const & iAngle
			, double const & min
			, double const & max
			)
		{
			double oAngle{ iAngle };
			// check for under value
			if (oAngle < min)
			{
				oAngle += piTwo;
			}
			else
			// check for over value
			if (! (oAngle < max))
			{
				oAngle -= piTwo;
			}
			return oAngle;
		}

	public:

		//! Construct accumulation grid with dimension adSize
		inline
		explicit
		AdderAD
			( dat::SizeHW const & adSize
			)
			: theSizeMap
				( adSize 
				, dat::Span{ -pi, pi }
				, dat::Span{ 0., piTwo }
				)
			, theGridAD(adSize)
		{
			std::fill(theGridAD.begin(), theGridAD.end(), 0.f);
		}

		//! Direct access to the accumulation grid
		inline
		dat::Grid<float> const &
		grid
			() const
		{
			return theGridAD;
		}

		//! Row/column in accumulation grid associated with ParmAD values
		inline
		dat::Spot
		datSpotForAD
			( hough::ParmAD const & parmAD
			) const
		{
			// Get angles from parmAD and ensure within valid sizemap range
			dat::Spot const parmSpot
				{ angleInRange(parmAD.alpha(), -pi, pi)
				, angleInRange(parmAD.delta(), 0., piTwo)
				};
			return dat::Spot{ theSizeMap.gridSpotForAreaSpot(parmSpot) };
		}

		//! Row/column in accumulation grid associated with ParmAD values
		inline
		dat::RowCol
		datRowColForAD
			( hough::ParmAD const & parmAD
			) const
		{
			dat::Spot const gridSpot{ datSpotForAD(parmAD) };
			return cast::datRowCol(gridSpot);
		}

		//! Hough alpha,delta parameter values at location gridSpot
		inline
		hough::ParmAD
		houghParmADFor
			( dat::Spot const & gridSpot
			) const
		{
			hough::ParmAD parmAD{};
			if ( gridSpot.isValid()
			  && ( gridSpot[0] < (double)theGridAD.high())
			  && ( gridSpot[1] < (double)theGridAD.wide())
			   )
			{
				dat::Spot const areaSpot
					{ theSizeMap.areaSpotForGridSpot(gridSpot) };
				parmAD = hough::ParmAD{ areaSpot[0], areaSpot[1] };
			}
			return parmAD;
		}

		//! Bell curve like weighting function exp(-|fracSpot|^2).
		inline
		double
		weightAt
			( dat::Vec2D<double> const & fracSpot
			, double const & sigma = 1.
			)
		{
			double const radius{ magnitude(fracSpot) };
			double const zVal{ radius / sigma };
			return std::exp(-zVal*zVal);
		}

		//! Incorporate parmAD values into theGridAD with gridMag as weight
		inline
		void
		add
			( hough::ParmAD const & parmAD
			, float const & gradMag
			)
		{
			if (pix::isValid(gradMag) && parmAD.isValid())
			{

/*
s/LT/NN/g
s/MT/ZN/g
s/RT/PN/g
s/LM/NZ/g
s/MM/ZZ/g
s/RM/PZ/g
s/LB/NP/g
s/MB/ZP/g
s/RB/PP/g
*/

				static dat::Spot const dSpotLT{ -1., -1. };
				static dat::Spot const dSpotMT{  0., -1. };
				static dat::Spot const dSpotRT{  1., -1. };

				static dat::Spot const dSpotLM{ -1.,  0. };
				static dat::Spot const dSpotMM{  0.,  0. };
				static dat::Spot const dSpotRM{  1.,  0. };

				static dat::Spot const dSpotLB{ -1.,  1. };
				static dat::Spot const dSpotMB{  0.,  1. };
				static dat::Spot const dSpotRB{  1.,  1. };


				dat::Spot const spot00{ datSpotForAD(parmAD) };

				static dat::Spot const spotLT{ spot00 + dSpotLT };
				static dat::Spot const spotMT{ spot00 + dSpotMT };
				static dat::Spot const spotRT{ spot00 + dSpotRT };

				static dat::Spot const spotLM{ spot00 + dSpotLM };
				static dat::Spot const spotMM{ spot00 + dSpotMM };
				static dat::Spot const spotRM{ spot00 + dSpotRM };

				static dat::Spot const spotLB{ spot00 + dSpotLB };
				static dat::Spot const spotMB{ spot00 + dSpotMB };
				static dat::Spot const spotRB{ spot00 + dSpotRB };


				dat::RowCol const rcLT{ cast::datRowCol(spotLT) };
				dat::RowCol const rcMT{ cast::datRowCol(spotMT) };
				dat::RowCol const rcRT{ cast::datRowCol(spotRT) };

				dat::RowCol const rcLM{ cast::datRowCol(spotLM) };
				dat::RowCol const rcMM{ cast::datRowCol(spotMM) };
				dat::RowCol const rcRM{ cast::datRowCol(spotRM) };

				dat::RowCol const rcLB{ cast::datRowCol(spotLB) };
				dat::RowCol const rcMB{ cast::datRowCol(spotMB) };
				dat::RowCol const rcRB{ cast::datRowCol(spotRB) };

/*
std::cout << '\n';

std::cout << "rcLT: " << rcLT << '\n';
std::cout << "rcMT: " << rcMT << '\n';
std::cout << "rcRT: " << rcRT << '\n';

std::cout << "rcLM: " << rcLM << '\n';
std::cout << "rcMM: " << rcMM << '\n';
std::cout << "rcRM: " << rcRM << '\n';

std::cout << "rcLB: " << rcLB << '\n';
std::cout << "rcMB: " << rcMB << '\n';
std::cout << "rcRB: " << rcRB << '\n';
*/


				dat::Spot const fracMM{ spotMM - cast::datSpot(rcMM) };

				double const weightLT{ weightAt(fracMM + dSpotLT) };
				double const weightMT{ weightAt(fracMM + dSpotMT) };
				double const weightRT{ weightAt(fracMM + dSpotRT) };

				double const weightLM{ weightAt(fracMM + dSpotLM) };
				double const weightMM{ weightAt(fracMM + dSpotMM) };
				double const weightRM{ weightAt(fracMM + dSpotRM) };

				double const weightLB{ weightAt(fracMM + dSpotLB) };
				double const weightMB{ weightAt(fracMM + dSpotMB) };
				double const weightRB{ weightAt(fracMM + dSpotRB) };


				theGridAD(rcLT) += weightLT * gradMag;
				theGridAD(rcMT) += weightMT * gradMag;
				theGridAD(rcRT) += weightRT * gradMag;

				theGridAD(rcLM) += weightLM * gradMag;
				theGridAD(rcMM) += weightMM * gradMag;
				theGridAD(rcRM) += weightRM * gradMag;

				theGridAD(rcLB) += weightLB * gradMag;
				theGridAD(rcMB) += weightMB * gradMag;
				theGridAD(rcRB) += weightRB * gradMag;

			}
		}

	}; // AdderAD


} // [hough]

} // [quadloco]

