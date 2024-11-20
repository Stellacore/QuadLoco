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
#include "datMapSizeArea.hpp"
#include "datRowCol.hpp"
#include "datSizeHW.hpp"
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

		dat::MapSizeArea const theMapSizeArea;
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
			: theMapSizeArea
				( adSize 
				, dat::Area
					{ dat::Span{ -pi, pi }
					, dat::Span{ 0., piTwo }
					}
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
			return dat::Spot{ theMapSizeArea.gridSpotForAreaSpot(parmSpot) };
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
					{ theMapSizeArea.areaSpotForGridSpot(gridSpot) };
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

				dat::Spot const spot00{ datSpotForAD(parmAD) };

				static dat::Spot const dSpotNN{ -1., -1. };
				static dat::Spot const dSpotZN{  0., -1. };
				static dat::Spot const dSpotPN{  1., -1. };

				static dat::Spot const dSpotNZ{ -1.,  0. };
				static dat::Spot const dSpotZZ{  0.,  0. };
				static dat::Spot const dSpotPZ{  1.,  0. };

				static dat::Spot const dSpotNP{ -1.,  1. };
				static dat::Spot const dSpotZP{  0.,  1. };
				static dat::Spot const dSpotPP{  1.,  1. };


				static dat::Spot const spotNN{ spot00 + dSpotNN };
				static dat::Spot const spotZN{ spot00 + dSpotZN };
				static dat::Spot const spotPN{ spot00 + dSpotPN };

				static dat::Spot const spotNZ{ spot00 + dSpotNZ };
				static dat::Spot const spotZZ{ spot00 + dSpotZZ };
				static dat::Spot const spotPZ{ spot00 + dSpotPZ };

				static dat::Spot const spotNP{ spot00 + dSpotNP };
				static dat::Spot const spotZP{ spot00 + dSpotZP };
				static dat::Spot const spotPP{ spot00 + dSpotPP };


				dat::RowCol const rcNN{ cast::datRowCol(spotNN) };
				dat::RowCol const rcZN{ cast::datRowCol(spotZN) };
				dat::RowCol const rcPN{ cast::datRowCol(spotPN) };

				dat::RowCol const rcNZ{ cast::datRowCol(spotNZ) };
				dat::RowCol const rcZZ{ cast::datRowCol(spotZZ) };
				dat::RowCol const rcPZ{ cast::datRowCol(spotPZ) };

				dat::RowCol const rcNP{ cast::datRowCol(spotNP) };
				dat::RowCol const rcZP{ cast::datRowCol(spotZP) };
				dat::RowCol const rcPP{ cast::datRowCol(spotPP) };

/*
std::cout << '\n';

std::cout << "rcNN: " << rcNN << '\n';
std::cout << "rcZN: " << rcZN << '\n';
std::cout << "rcPN: " << rcPN << '\n';

std::cout << "rcNZ: " << rcNZ << '\n';
std::cout << "rcZZ: " << rcZZ << '\n';
std::cout << "rcPZ: " << rcPZ << '\n';

std::cout << "rcNP: " << rcNP << '\n';
std::cout << "rcZP: " << rcZP << '\n';
std::cout << "rcPP: " << rcPP << '\n';
*/


				dat::Spot const fracZZ{ spotZZ - cast::datSpot(rcZZ) };

				double const weightNN{ weightAt(fracZZ + dSpotNN) };
				double const weightZN{ weightAt(fracZZ + dSpotZN) };
				double const weightPN{ weightAt(fracZZ + dSpotPN) };

				double const weightNZ{ weightAt(fracZZ + dSpotNZ) };
				double const weightZZ{ weightAt(fracZZ + dSpotZZ) };
				double const weightPZ{ weightAt(fracZZ + dSpotPZ) };

				double const weightNP{ weightAt(fracZZ + dSpotNP) };
				double const weightZP{ weightAt(fracZZ + dSpotZP) };
				double const weightPP{ weightAt(fracZZ + dSpotPP) };

// TODO - need to wrap row/col coordinates back into the grid!


				theGridAD(rcNN) += weightNN * gradMag;
				theGridAD(rcZN) += weightZN * gradMag;
				theGridAD(rcPN) += weightPN * gradMag;

				theGridAD(rcNZ) += weightNZ * gradMag;
				theGridAD(rcZZ) += weightZZ * gradMag;
				theGridAD(rcPZ) += weightPZ * gradMag;

				theGridAD(rcNP) += weightNP * gradMag;
				theGridAD(rcZP) += weightZP * gradMag;
				theGridAD(rcPP) += weightPP * gradMag;

			}
		}

	}; // AdderAD


} // [hough]

} // [quadloco]

